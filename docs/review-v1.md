# Review code v1 — nguyên nhân mất kết nối BLE

Triệu chứng: kết nối giữa Pico W và Technic Hub không ổn định, thỉnh thoảng mất kết nối và phải khởi động lại cả xe.

Các vấn đề dưới đây xếp theo mức độ ảnh hưởng. Mọi vấn đề đều đã được sửa trong v2 (commit `38c79d7`), trừ khi có ghi chú khác.

## A. Nguyên nhân gây mất kết nối

### 1. Gọi BTstack/CYW43 từ ngữ cảnh ngắt (nghiêm trọng nhất)
`add_repeating_timer_ms` chạy callback trong IRQ của hardware alarm. Trong callback, v1 gọi `cyw43_arch_poll()`, các hàm ghi GATT và `printf`. Cùng lúc đó, `btstack_run_loop_execute()` ở main cũng poll cùng chip CYW43 và cùng BTstack. Hai luồng thực thi chen ngang nhau trên một stack không thread-safe và trên bus SPI tới chip radio, gây mất link hoặc treo Pico một cách ngẫu nhiên.

**Sửa:** control loop chạy bằng `btstack_timer_source_t` trong run loop của BTstack và không gọi `cyw43_arch_poll()` thủ công nữa.

### 2. Không có flow control khi gửi lệnh
Mỗi chu kỳ 25 Hz gửi 3 lệnh, tổng cộng 75 gói/s, gói nào cũng yêu cầu command feedback (cờ `0x11`). Return code của hàm ghi bị bỏ qua, nên khi buffer ACL đầy (BTstack chỉ dùng 3 buffer để tránh tràn bus CYW43) thì lệnh mất âm thầm. Gửi `GotoAbsolutePosition` mỗi 40 ms còn làm motor khởi động lại profile liên tục, khiến lái bị rung.

**Sửa:** chỉ gửi khi giá trị đổi và refresh mỗi 300 ms, kiểm tra return code, dùng cờ `0x10`.

### 3. Supervision timeout mặc định quá ngắn
Mặc định của BTstack (đã kiểm tra trong `hci.c`) là supervision timeout 720 ms với latency 4.

**Sửa:** supervision timeout 2 s, latency 0, interval 15–30 ms.

### 4. State machine không có timeout
Pico có thể kẹt mãi ở `CONNECTING` hoặc `W4_SERVICES`/`W4_CHARS`. Trong lúc đó hub quảng bá một thời gian, không có ai kết nối thì tự tắt. Đây là lý do phải khởi động lại cả xe.

**Sửa:** timeout 8 s cho mỗi bước setup (`gap_connect_cancel` / `gap_disconnect`, sau đó quét lại) và hardware watchdog 1 s.

### 5. Motor lái có thể kẹt ở endstop và gây sụt áp
Cấu hình 40° × 6 = 240° góc motor, `MAXPOWER 100`, end state HOLD. Nếu góc này vượt hành trình thanh răng, motor ghì vào endstop ở công suất tối đa. Cộng với hai motor kéo chạy 100%, pin AA sụt áp và hub có thể reset.

**Sửa một phần:** thêm `STEER_MOTOR_DEG_LIMIT` và giảm max power xuống 80; bật cảnh báo low voltage/high current và báo % pin. **Cần đo góc endstop thực tế** để đặt giới hạn cho đúng.

## B. An toàn và logic

### 6. Glitch PWM có thể ra lệnh lùi hết ga
Khi IRQ GPIO bị trễ (do lỗi #1 chiếm CPU), cờ RISE và FALL được chốt cùng lúc. v1 xử lý RISE trước (`rise_us = t`), sau đó FALL tính ra `dt = 0`, giá trị này lọt qua kiểm tra `dt < 3000`. Kết quả `pulse_us = 0`, map thành throttle −100.

**Sửa:** bỏ sự kiện có cả hai cạnh cùng lúc; chỉ chấp nhận xung 800–2200 µs. Về lâu dài nên đo xung bằng PIO.

### 7. "BLE disconnect → STOP" không thực hiện được từ gateway
Khi link đã mất, Pico không gửi được lệnh dừng. An toàn phụ thuộc vào cách hub xử lý lệnh `StartPower` cuối cùng. **Cần test thực tế**: giữ ga, rút điện Pico, rồi quan sát.

### 8. Điểm 0 của motor lái là vị trí lúc bật hub
`GotoAbsolutePosition` lấy encoder lúc hub khởi động làm gốc. Bật xe khi bánh đang lệch thì tâm lệch theo. **Chưa sửa**: cần routine calibration (chạm hai endstop, lấy trung điểm).

### 9. Các điểm nhỏ
- **Heuristic discovery dễ vỡ.** v2 so khớp đúng characteristic `00001624-1212-EFDE-1623-785FEABCD123`; BTstack lưu UUID128 theo big-endian. Heuristic được giữ làm dự phòng.
- **Buffer trên stack cho ghi có phản hồi.** Nhánh `WRITE` truyền buffer `msg` trên stack, trong khi BTstack giữ con trỏ đến khi ghi xong. Ngoài ra, một lần ghi lỗi ở trạng thái READY làm `gap_disconnect`.
- **Race khi tính tuổi xung.** `pwm_fresh` đọc `now` trước `last_update_us`, nên có thể underflow.
- **`start_scan()` dự phòng sau `sleep_ms(1000)` là thừa.**

## C. Tài liệu và code không khớp
Code v1 gốc không có `STEER_GEAR_RATIO` và dùng 35/60/80, trong khi tài liệu ghi 40/6/80/100. v2 dùng gear ratio 6 và max power 80.

## Kiểm chứng
- v1 và v2 đều build được với Pico SDK 2.2.0. v2 build sạch với `-Wall`.
- Đã đối chiếu với mã nguồn BTstack: thứ tự byte UUID128, chữ ký `gap_set_connection_parameters`/`gap_connect_cancel`, và giá trị mặc định 720 ms.
- **Chưa test trên phần cứng.**
