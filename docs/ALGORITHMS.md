# Giải Thuật và Trí Tuệ Nhân Tạo (Algorithms & AI)

Tài liệu này giải thích sâu về các lý thuyết và thuật toán cốt lõi giúp chiếc xe lăn trở nên "thông minh".

## 1. Trí Tuệ Nhân Tạo (AI & Computer Vision)

### Kiến Trúc Mạng CNN 11 Lớp
Hệ thống sử dụng mạng nơ-ron tích chập (Convolutional Neural Network - CNN) tùy chỉnh để xử lý hình ảnh đầu vào và phân loại đối tượng/điều hướng.

**Cấu trúc chi tiết:**
1.  **Input Layer:** Ảnh màu RGB (Resize về kích thước chuẩn, ví dụ 66x200 hoặc 128x128).
2.  **Conv2D Layers (5 lớp đầu):**
    *   Trích xuất đặc trưng (Feature Extraction).
    *   Sử dụng hàm kích hoạt **ReLU** để tăng tính phi tuyến tính.
    *   *Mục đích:* Nhận diện cạnh, góc, màu sắc, hình dáng người/biển báo.
3.  **Flatten Layer:** Làm phẳng dữ liệu từ mảng đa chiều sang vector 1 chiều.
4.  **Dense Layers (Fully Connected - 5 lớp sau):**
    *   Các lớp kết nối đầy đủ (100 -> 50 -> 10 nơ-ron).
    *   *Mục đích:* Ra quyết định dựa trên đặc trưng đã trích xuất.
5.  **Output Layer:**
    *   *Classification:* Softmax (Xác suất là người/vật cản/đường đi).
    *   *Regression (End-to-End):* 1 Node duy nhất (Góc lái - Steering Angle).

### Tính Năng Follow-me
*   **Logic:** Sử dụng Bounding Box quy đổi từ AI (hoặc YOLO).
*   **Điều khiển:**
    *   *Heading Control:* Giữ tâm Bounding Box ở giữa khung hình (x_center ~ image_width/2).
    *   *Distance Control:* Giữ diện tích Bounding Box (Box Size) ở mức ổn định. Nếu Box nhỏ đi -> Xe tăng tốc tiến tới.

---

## 2. Hệ Thống Điều Khiển (PID Control System)

Để xe di chuyển mượt mà và không bị rung lắc (Jerk), hệ thống áp dụng bộ điều khiển PID cho từng bánh xe.

**Công thức:**
$$ Output(t) = K_p e(t) + K_i \int_{0}^{t} e(\tau)d\tau + K_d \frac{de(t)}{dt} $$

Trong đó:
*   **$SP$ (Set Point):** Vận tốc mục tiêu (từ ROS gửi xuống).
*   **$PV$ (Process Variable):** Vận tốc thực tế (đọc từ Encoder).
*   **$e(t) = SP - PV$:** Sai số.

| Thành Phần | Vai Trò Trong Xe Lăn |
| :--- | :--- |
| **$K_p$ (Proportional)** | Phản ứng nhanh với sai số. $K_p$ lớn giúp xe đạt tốc độ nhanh nhưng dễ gây vọt lố (overshoot). |
| **$K_i$ (Integral)** | Cộng dồn sai số theo thời gian. Giúp xe đạt đúng tốc độ mục tiêu chính xác (triệt tiêu sai số tĩnh) khi lên dốc/xuống dốc tải trọng thay đổi. |
| **$K_d$ (Derivative)** | Dự đoán tương lai của sai số. Giúp giảm dao động, làm mượt chuyển động phanh/tăng tốc, chống rung. |

---

## 3. Localization & Mapping (SLAM)

### SLAM (Simultaneous Localization and Mapping)
*   Xe sử dụng Lidar để quét môi trường và xây dựng bản đồ 2D (Occupancy Grid Map).
*   **Thuật toán:** Gmapping (Filter-based) hoặc Cartographer (Graph-based).
*   **Đầu ra:** File bản đồ `.pgm` (hình ảnh) và `.yaml` (metadata).

### AMCL (Adaptive Monte Carlo Localization)
*   Khi đã có bản đồ, xe sử dụng AMCL để định vị.
*   **Cơ chế:** Rải các "hạt" (particles) giả thuyết lên bản đồ. Khi Lidar quét trúng vật cản khớp với bản đồ, các hạt tại vị trí đó được tăng trọng số. Các hạt sai vị trí bị loại bỏ. Dần dần, đám mây hạt hội tụ về vị trí thực của xe.

---

## 4. Local Planner (DWA)
Trong Navigation Stack, **DWA (Dynamic Window Approach)** được sử dụng để né vật cản động.
*   Thuật toán mô phỏng nhiều quỹ đạo (trajectories) khả thi trong một cửa sổ thời gian ngắn.
*   Đánh giá từng quỹ đạo dựa trên các tiêu chí: Khoảng cách tới đích, khoảng cách tới vật cản, vận tốc xe.
*   Chọn quỹ đạo có điểm số cao nhất để thực thi.
