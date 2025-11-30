import cv2
import numpy as np
import pyrealsense2 as rs
import easyocr
import requests
import time
import threading
from PIL import Image, ImageDraw, ImageFont
from queue import Queue, Empty

# =====================================
# 1. DeepL
# =====================================
DEEPL_API_KEY = "1e8f1e6d-24b6-498f-9cc4-0d0ad9f8110d:fx"

CANNY_LOW  = 50
CANNY_HIGH = 150
MIN_AREA = 20000
MAX_AREA = 200000
OCR_INTERVAL = 0.5
ALPHA = 0.3

missing_frame_count = 0 
MAX_MISSING_FRAMES = 5

# 번역 요청/캐시
translate_queue = Queue()
translation_cache = {}      # { full_eng(str): kor(str) }
requested_texts  = set()    # 이미 요청 넣은 영어 문장들


# =====================================
# DeepL 번역 스레드
# =====================================
def translate(text_eng: str) -> str:
    if not text_eng.strip():
        return "(텍스트 없음)"

    url = "https://api-free.deepl.com/v2/translate"
    headers = {"Authorization": f"DeepL-Auth-Key {DEEPL_API_KEY}"}
    data = {"text": text_eng, "source_lang": "EN", "target_lang": "KO"}

    try:
        # 🔥 타임아웃을 5.0초로 유지
        r = requests.post(url, headers=headers, data=data, timeout=5.0) 
        r.raise_for_status()
        j = r.json()
        t = j.get("translations", [])
        if not t:
            # DeepL에서 정상 응답했으나 번역 결과가 없을 경우
            print(f"[DeepL ERROR] No translation found for: {text_eng[:30]}...")
            return "(번역 결과 없음)"
        return t[0].get("text", "(번역 오류)")
    except requests.exceptions.RequestException as e:
        # 네트워크 오류, 타임아웃, HTTP 오류(4xx, 5xx) 등
        print(f"[DeepL API ERROR] Request failed for '{text_eng[:30]}...': {e}")
        return "(번역 오류)"
    except Exception as e:
        # 기타 JSON 파싱 오류 등
        print(f"[DeepL API ERROR] An unexpected error occurred: {e}")
        return "(번역 오류)"


def translate_thread():
    global translation_cache
    while True:
        try:
            # 큐에서 작업 가져오기
            text_eng = translate_queue.get(timeout=1)
        except Empty:
            continue
        
        # 번역 시도
        trans = translate(text_eng)
        
        # 번역 결과를 캐시에 저장
        translation_cache[text_eng] = trans

        # 큐 작업 완료 보고
        translate_queue.task_done()


# 큐 작업 스레드 시작
threading.Thread(target=translate_thread, daemon=True).start()


# =====================================
# 사각형 warp 함수
# =====================================
def order_points(pts):
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1)

    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect


def warp_quad(image, quad):
    pts = quad.reshape(4, 2).astype("float32")
    rect = order_points(pts)

    (tl, tr, br, bl) = rect
    widthA = np.linalg.norm(br - bl)
    widthB = np.linalg.norm(tr - tl)
    maxW = int(max(widthA, widthB))

    heightA = np.linalg.norm(tr - br)
    heightB = np.linalg.norm(tl - bl)
    maxH = int(max(heightA, heightB))
    maxW = max(maxW, 40)
    maxH = max(maxH, 40)

    dst = np.array([[0,0],[maxW-1,0],[maxW-1,maxH-1],[0,maxH-1]], dtype="float32")
    M = cv2.getPerspectiveTransform(rect, dst)
    warped = cv2.warpPerspective(image, M, (maxW, maxH))

    return warped, M, rect, dst


# =====================================
# 🔥 문장 단위 클러스터링
# =====================================
def group_words(results):
    if len(results) == 0:
        return []

    words = []
    for r in results:
        bbox, text, _ = r
        pts = np.array(bbox)

        # bbox 상단 y 평균
        top_y = int((pts[0][1] + pts[1][1]) / 2)
        # 좌우 순서 안정화 → 최소 x
        min_x = int(np.min(pts[:,0]))

        words.append((min_x, top_y, bbox, text))

    # 줄 순으로 y 정렬
    words.sort(key=lambda x: x[1])

    clusters = []
    current = [words[0]]

    LINE_GAP = 20
    WORD_GAP = 130

    for i in range(1, len(words)):
        prev = current[-1]
        curr = words[i]

        # y축 차이가 작으면 같은 줄
        if abs(prev[1] - curr[1]) < LINE_GAP:
            # x축 차이가 작으면 같은 문장
            if abs(prev[0] - curr[0]) < WORD_GAP:
                current.append(curr)
            else:
                clusters.append(current)
                current = [curr]
        else:
            clusters.append(current)
            current = [curr]

    clusters.append(current)

    # 각 클러스터 내 x 정렬
    for c in clusters:
        c.sort(key=lambda x: x[0])

    return clusters


# =====================================
# overlay 생성 (warp 공간)
# =====================================
def create_overlay(h, w, word_states, warped_img):
    """
    word_states: { key: {"eng":..., "kor":..., "bbox":[[x,y],...]} }
    warped_img: 현재 AR 처리가 진행 중인 원본 영역 (warped) 이미지
    """
    # [AR 배경] 흰색 반투명 오버레이를 위해 np.ones * 255 사용 (사용자 설정 유지)
    overlay = np.ones((h, w, 3), dtype=np.uint8) * 255
    pil_img = Image.fromarray(overlay)
    draw = ImageDraw.Draw(pil_img)

    for key, st in word_states.items():
        kor = st["kor"]
        bbox = np.array(st["bbox"])

        # bbox 영역
        min_x = int(np.min(bbox[:,0]))
        max_x = int(np.max(bbox[:,0]))
        min_y = int(np.min(bbox[:,1]))
        max_y = int(np.max(bbox[:,1]))
        
        # 🔥 1. 텍스트 BBOX의 원본 이미지 픽셀 평균 밝기 계산 (warped_img 사용)
        # ------------------------------------------------------------------
        text_roi = warped_img[min_y:max_y, min_x:max_x] 

        # 텍스트 영역의 평균 밝기 (grayscale) 계산
        if text_roi.size > 0:
            # BGR을 Gray로 변환 후 평균 계산
            gray_roi = cv2.cvtColor(text_roi, cv2.COLOR_BGR2GRAY)
            avg_brightness = np.mean(gray_roi)
        else:
            # ROI가 너무 작거나 없을 경우 대비 (중간 밝기)
            avg_brightness = 128
            
        # 2. 밝기에 따라 텍스트 색상 결정 (고대비)
        # 임계값 30: 사용자의 피드백에 따라 임계값을 30으로 설정하여, 
        #           매우 어두운 배경(밝기 <= 30)이 아니면 흰색 텍스트를 적극적으로 사용합니다.
        CONTRAST_THRESHOLD = 30 
        if avg_brightness > CONTRAST_THRESHOLD:
            text_color = (0, 0, 0)   # Black (검은색)
        else:
            text_color = (255, 255, 255) # White (흰색)
        # ------------------------------------------------------------------

        # OCR 글씨 높이
        height = max_y - min_y
        if height < 5:
            height = 5

        # 1) 번역 글씨 크기 (글씨 높이에 비례)
        font_size = int(height * 0.5)
        font_size = max(font_size, 10) # 최소 폰트 크기 10pt 보장
        
        try:
            # NotoSansCJK-Regular.ttc 폰트 사용
            font = ImageFont.truetype(
                "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
                font_size
            )
        except:
            font = ImageFont.load_default()

        # 2) 번역 글씨 위치 (글씨 아래 + 높이 * 0.7)
        text_y = min(max_y + int(height * 0.5), h - height) 

        text = kor
        tb = draw.textbbox((0,0), text, font=font)
        tw = tb[2] - tb[0]

        text_x = min_x + (max_x - min_x)//2 - tw//2

        # 텍스트 그리기: 계산된 text_color 사용
        draw.text((text_x, text_y), text, fill=text_color, font=font)

    return np.array(pil_img)

# main 함수 밖에 정의 (혹은 main 함수 안에 nested function으로 정의)
def select_and_translate_roi(frame, reader):
    """
    프레임에서 ROI를 선택하고, 해당 영역의 텍스트를 OCR 및 번역 후 팝업으로 표시
    (수정: 원본 텍스트 위치에 번역 텍스트를 오버레이합니다.)
    """
    # 1. 원본 프레임 복사
    temp_frame = frame.copy()
    
    # 2. ROI 선택 창 표시
    win_name = "Select ROI for Translation"
    cv2.namedWindow(win_name)
    
    # 선택 영역 좌표 (x, y, w, h)
    ret = cv2.selectROI(win_name, temp_frame)
    cv2.destroyWindow(win_name) # ROI 선택 후 창 닫기
    
    x, y, w, h = ret
    
    if w > 0 and h > 0:
        # 3. ROI 영역 잘라내기
        roi_img = frame[int(y):int(y+h), int(x):int(x+w)]
        
        # 4. OCR 수행 (원본 크기 이미지 사용)
        print("[ROI OCR] Running OCR on selected area...")
        roi_gray = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
        
        # EasyOCR 실행 (WARP 없이 바로 실행)
        # 결과: [[bbox, text, confidence], ...]
        results = reader.readtext(roi_gray, detail=1) 
        
        if not results:
            print("[ROI OCR] No text found in ROI.")
            return

        # 5. 텍스트 그룹화 및 번역 요청
        clusters = group_words(results) # 기존 함수 활용
        
        # 5-1. 클러스터별로 영어 문장을 추출하고 DeepL 요청을 위한 텍스트를 만듦
        all_eng_sentences = []
        for cluster in clusters:
            full_eng = " ".join([word[3] for word in cluster]).strip()
            if full_eng:
                all_eng_sentences.append(full_eng)
                
        full_eng_text = "\n".join(all_eng_sentences)

        # 6. 동기 번역 (DeepL 요청) - 동기 호출은 오류 발생 시 팝업에 즉시 반영됨
        if not full_eng_text:
            kor_translation = "(텍스트 없음)"
        else:
            print("[ROI OCR] Found: '{}'".format(full_eng_text.replace('\n', ' ')))
            kor_translation = translate(full_eng_text)
        
        # 6-1. 번역된 텍스트를 줄 단위로 분리하여 클러스터와 매핑 준비
        kor_lines = kor_translation.split('\n')
        
        # 7. 번역 결과 팝업 표시
        
        # --- 7-1. 이미지 확대 계산 및 적용 ---
        h_orig, w_orig = roi_img.shape[:2]
        MIN_DIMENSION = 360 # 최소 200픽셀 보장
        
        scale = 1.0
        
        # 원본 ROI 이미지의 크기가 최소 크기(200)보다 작을 경우에만 확대
        if w_orig < MIN_DIMENSION or h_orig < MIN_DIMENSION:
            scale_w = MIN_DIMENSION / w_orig
            scale_h = MIN_DIMENSION / h_orig
            scale = max(scale_w, scale_h) 

            new_w = int(w_orig * scale)
            new_h = int(h_orig * scale)
            
            resized_roi_img = cv2.resize(roi_img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        else:
            resized_roi_img = roi_img.copy()
        
        # 7-2. 오버레이 합성 준비 (확대된 이미지 기반)
        pil_roi_img = Image.fromarray(cv2.cvtColor(resized_roi_img, cv2.COLOR_BGR2RGB)).convert('RGBA')

        # 🔥 [핵심 수정] 반투명 배경과 텍스트를 그릴 투명 레이어 생성
        overlay_layer = Image.new('RGBA', pil_roi_img.size, (0, 0, 0, 0))
        draw = ImageDraw.Draw(overlay_layer)
        
        TRANSPARENCY_BKG = 180 # 180/255 = 약 70% 불투명 (사용자가 80을 시도했으므로 180은 눈에 띄게 반투명함)
        PADDING = 5            # 텍스트 잘림 방지를 위해 배경 박스에 여백 추가

        # 클러스터 순회 및 오버레이
        for i, cluster in enumerate(clusters):
            # 문장별 번역 결과 매핑 (번역 오류 시에도 해당 오류 메시지 사용)
            kor_text = kor_lines[i] if i < len(kor_lines) else "(번역 오류)"
            
            # 1. 문장 전체 BBox 계산 (원본 ROI 이미지 기준 좌표)
            all_pts = []
            for _, _, bbox, _ in cluster:
                all_pts.extend(bbox)
            all_pts = np.array(all_pts)
            
            orig_min_x = int(np.min(all_pts[:, 0]))
            orig_max_x = int(np.max(all_pts[:, 0]))
            orig_min_y = int(np.min(all_pts[:, 1]))
            orig_max_y = int(np.max(all_pts[:, 1]))
            
            # 2. 확대 비율 적용하여 확대된 이미지 위에서의 BBox 좌표 계산
            scaled_min_x = int(orig_min_x * scale)
            scaled_max_x = int(orig_max_x * scale)
            scaled_min_y = int(orig_min_y * scale)
            scaled_max_y = int(orig_max_y * scale)
            
            # 3. 반투명 배경 (OCR BBox 영역을 덮음) - PADDING 적용
            # 배경은 별도의 투명 레이어(overlay_layer)에 그립니다.
            draw.rectangle([
                (scaled_min_x - PADDING, scaled_min_y - PADDING), 
                (scaled_max_x + PADDING, scaled_max_y + PADDING)
            ], fill=(0, 0, 0, TRANSPARENCY_BKG)) 
            
            # 4. 텍스트 폰트 설정
            bbox_height = scaled_max_y - scaled_min_y
            current_font_size = int(bbox_height * 0.4) 
            current_font_size = max(current_font_size, 15)
            
            try:
                font = ImageFont.truetype(
                    "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
                    current_font_size
                )
            except:
                font = ImageFont.load_default()

            # 5. 텍스트 위치 계산 (BBox 중앙)
            tb = draw.textbbox((0, 0), kor_text, font=font)
            tw = tb[2] - tb[0]
            th = tb[3] - tb[1]
            
            # BBox 중앙에 텍스트 위치
            text_pos_x = scaled_min_x + (scaled_max_x - scaled_min_x) // 2 - tw // 2
            text_pos_y = scaled_min_y + (scaled_max_y - scaled_min_y) // 2 - th // 2
            
            # 6. 텍스트 그리기 (흰색으로 고정)
            # 텍스트도 별도의 투명 레이어(overlay_layer)에 그립니다.
            draw.text((text_pos_x, text_pos_y), kor_text, fill=(255, 255, 255), font=font)

        # 🔥 7-4. 원본 이미지와 반투명 배경/텍스트 레이어를 합성합니다.
        pil_roi_img = Image.alpha_composite(pil_roi_img, overlay_layer)
        
        # 7-5. PIL 이미지를 다시 OpenCV 형식으로 변환 (RGBA->BGR)
        result_img_cv = cv2.cvtColor(np.array(pil_roi_img), cv2.COLOR_RGBA2BGR)
        # ----------------------------------------------------

        # 7-6. 결과 창 표시 및 대기
        cv2.namedWindow("ROI Translation Result", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("ROI Translation Result", result_img_cv)
        
        # [핵심] 사용자가 닫을 때까지 대기 (스레드에서 실행되므로 메인 영상은 안 멈춤)
        cv2.waitKey(0) 
        
        # 창 닫기
        cv2.destroyWindow("ROI Translation Result")
        cv2.destroyWindow(win_name)

# =====================================
# 5. 메인 (여러 장 A4 지원) [수정됨]
# =====================================
def main():
    global missing_frame_count, MAX_MISSING_FRAMES
    print("Initializing EasyOCR...")
    reader = easyocr.Reader(['en'])

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    print("RealSense started.")

    last_overlay = None
    last_mask = None
    last_ocr_time = 0.0

    while True:
        frames = pipeline.wait_for_frames()
        f = frames.get_color_frame()
        if not f:
            continue

        frame = np.asanyarray(f.get_data())
        output = frame.copy()
        h, w = frame.shape[:2]
        now = time.time()

        # ----- Canny -----
        g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        g = clahe.apply(g)
        g_blur = cv2.GaussianBlur(g, (5,5), 0)
        edges = cv2.Canny(g_blur, CANNY_LOW, CANNY_HIGH)
        kernel = np.ones((5,5), np.uint8)
        edges = cv2.dilate(edges, kernel, 1)
        edges = cv2.erode(edges, kernel, 1)

        # ----- contour → 여러 개 quad 후보 -----
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        quads = []

        for cnt in contours:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02*peri, True)

            if len(approx) == 4:
                if not cv2.isContourConvex(approx):
                    continue
                area = cv2.contourArea(approx)
                if area < MIN_AREA or area > MAX_AREA:
                    continue
                hull = cv2.convexHull(cnt) # 원래 컨투어로 컨벡스 헐 계산
                hull_area = cv2.contourArea(hull)
                if hull_area == 0: # 0으로 나누는 오류 방지
                    continue
                solidity = float(area) / hull_area
                if solidity < 0.8: # 80% 미만이면 제외
                    continue
                x,y,w2,h2 = cv2.boundingRect(approx)
                asp = w2/float(h2)
                if 0.4 < asp < 2.0:
                    quads.append(approx)

        # ----- mask (여러 종이 union) -----
        mask = None

        if len(quads) > 0:
            # [성공] 감지되었으므로 카운터 리셋
            missing_frame_count = 0 
            
            mask = np.zeros((h, w), np.uint8)
            for q in quads:
                cv2.fillConvexPoly(mask, q.reshape(4,2), 255)
        
        else:
            # [실패] 감지 안 됨 -> 카운터 증가
            missing_frame_count += 1
            
            # [판단] 유예 기간(5프레임)을 넘겼는가?
            if missing_frame_count > MAX_MISSING_FRAMES:
                last_mask = None
                last_overlay = None
            else:
                # 5프레임이 안 지났으면, mask는 없어도(None)
                # last_overlay와 last_mask를 지우지 않고 그대로 둡니다.
                pass

        # ----- OCR + 번역 (여러 종이) -----
        if mask is not None and (now - last_ocr_time) > OCR_INTERVAL:
            overlay_total = np.zeros_like(frame, dtype=np.uint8)
            all_texts = []

            for quad in quads:
                warped, M, src, dst = warp_quad(frame, quad)
                warped_gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)

                results = reader.readtext(warped_gray, detail=1)

                clusters = group_words(results)

                # 이 종이 한 장에 대한 상태
                local_states = {}
                state_idx = 0

                for cluster in clusters:
                    full_eng = " ".join([word[3] for word in cluster]).strip()
                    if not full_eng:
                        continue

                    all_texts.append(full_eng)

                    # 문장 bbox
                    pts_all = []
                    for word in cluster:
                        pts_all.extend(word[2])
                    pts_all = np.array(pts_all)

                    bbox_sentence = [
                        [int(np.min(pts_all[:,0])), int(np.min(pts_all[:,1]))],
                        [int(np.max(pts_all[:,0])), int(np.min(pts_all[:,1]))],
                        [int(np.max(pts_all[:,0])), int(np.max(pts_all[:,1]))],
                        [int(np.min(pts_all[:,0])), int(np.max(pts_all[:,1]))]
                    ]

                    # 번역 캐시 조회 / 요청
                    kor = translation_cache.get(full_eng)
                    if kor is None:
                        kor = "(번역 대기중)"
                        if full_eng not in requested_texts:
                            translate_queue.put(full_eng)
                            requested_texts.add(full_eng)

                    state = {
                        "eng": full_eng,
                        "kor": kor,
                        "bbox": bbox_sentence
                    }
                    local_states[state_idx] = state
                    state_idx += 1

                # overlay 생성 (warp 공간)
                if len(local_states) > 0:
                    warp_h, warp_w = warped.shape[:2]
                    # [수정] warped 이미지를 인자로 전달
                    warped_overlay = create_overlay(warp_h, warp_w, local_states, warped)

                    # warp-back
                    Minv = cv2.getPerspectiveTransform(dst, src)
                    ov = cv2.warpPerspective(warped_overlay, Minv, (w, h))

                    # 여러 종이 overlay를 합성 (최댓값 사용)
                    overlay_total = np.maximum(overlay_total, ov)

            if all_texts:
                print("[OCR]", " | ".join(all_texts))
            else:
                print("[OCR] (no text)")

            last_overlay = overlay_total
            last_mask = mask
            last_ocr_time = now

        # ----- 반투명 합성 -----
        if last_overlay is not None and last_mask is not None:
            mask_f = (last_mask.astype(np.float32)/255.0)[...,None]
            out_f = output.astype(np.float32)
            ov_f = last_overlay.astype(np.float32)

            # ALPHA = 0.3 (30%) 불투명도로 원본 영상과 AR 오버레이 합성
            output = out_f*(1 - mask_f*ALPHA) + ov_f*(mask_f*ALPHA)
            output = output.astype(np.uint8)

        # 윤곽선 디버깅 (모든 종이)
        for q in quads:
            cv2.drawContours(output, [q], -1, (0,255,0), 3)

        cv2.imshow("RealSense AR Translate", output)
        cv2.imshow("Edges", edges)

        k = cv2.waitKey(1)&0xFF
        if k in [27, ord('q')]:
            break
        
        if k == ord('s'):
            # 메인 영상을 잠시 멈추고 현재 프레임 복사
            frame_to_process = frame.copy() 
            roi_thread = threading.Thread(
                target=select_and_translate_roi, 
                args=(frame_to_process, reader), 
                daemon=True # 메인 프로그램 종료 시 함께 종료되도록 설정
            )
            roi_thread.start()

    pipeline.stop()
    cv2.destroyAllWindows()


if __name__=="__main__":
    main()
