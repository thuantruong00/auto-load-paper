# Auto Load Paper

Relay: GPIO5. Lệnh `relay:0` = LOW, `relay:1` = HIGH, `reset` = bật relay `1` trong `500ms` rồi tắt. Dùng `status` để xem trạng thái.

Khi flow auto hoàn tất phần push, Serial in `cut` để báo cho app.

## Mapping

| Chức năng | Kết nối |
|---|---|
| Cảm biến giấy `DO` | ESP32 GPIO34 |
| Servo | ESP32 GPIO13 |
| Motor load qua MOSFET PWM | ESP32 GPIO25 |
| BTS7960 `RPWM` | ESP32 GPIO26 |
| BTS7960 `LPWM` | ESP32 GPIO27 |
| BTS7960 `R_EN`, `L_EN` | Nối trực tiếp 3.3V |
| PC817 sensor1 / opto1 | ESP32 GPIO19 |
| PC817 sensor2 / opto2 | ESP32 GPIO18 |
| Module relay `IN` | ESP32 GPIO5 |

Cảm biến giấy có debounce `100ms` và chỉ kích khi chuyển `CLEAR → ACTIVE`. Nếu lúc khởi động đang `ACTIVE`, hệ thống chờ cảm biến `CLEAR`. Sau khi đã nhận trigger, sensor có thể CLEAR trong thời gian chờ `500ms` mà flow vẫn tiếp tục.

## Chiều motor và công tắc hành trình

| Lệnh / chiều motor | BTS7960 | Công tắc giới hạn |
|---|---|---|
| `+` / tiến | `RPWM` GPIO26 | sensor1 / GPIO19 / opto1 |
| `-` / lùi | `LPWM` GPIO27 | sensor2 / GPIO18 / opto2 |

- PC817 dùng `INPUT_PULLUP`, mức tác động `LOW`.
- Serial in trạng thái `sensor1` và `sensor2`.
- Tín hiệu PC817 có debounce `50ms`.
- Khi auto push chạm sensor: dừng `700ms`, lùi `30%` trong `1500ms`, chờ `300ms`, re-approach `30%` đến khi sensor tác động lại, tối đa `3000ms`, rồi lùi thêm `30%` trong `600ms` để nhả sensor.
- Delay hiện tại: chạm sensor → bắt đầu lùi `700ms`; lùi xong → re-approach `300ms`; chạm lại sensor → nhả thêm `600ms` ở `30%`.
- Lệnh `home` chạy chiều `+100%` và dừng khi sensor2/GPIO18 tác động.
- Sau đó motor lùi `-30%` trong `300ms` để nhả sensor2 và in `Ready`.

## Flow tự động

1. Cảm biến giấy kích hoạt, chờ `500ms`.
2. Servo đến `141°`.
3. Motor load chạy `30%` trong `1500ms`.
4. Servo về `90°`, motor chạy thêm `1500ms` rồi dừng.
5. Push chạy tương đương `push_t:-100,11000`.
6. Kết thúc push: dừng và chờ lệnh `home`.
7. `home`: chạy `+100%`; sensor2/GPIO18 dừng motor, sau đó lùi `-30%` trong `300ms` và báo `Ready`.

## Lệnh Serial

Serial: `115200 baud`, chọn `New Line`.

| Lệnh | Chức năng |
|---|---|
| `push:<-100..100>` | Chạy liên tục; `0` dừng |
| `push_t:<speed>,<ms>` | Chạy có timeout, ví dụ `push_t:-100,11000` |
| `home` | Chạy về sensor2/GPIO18 ở `+100%`, tối đa `10000ms` |
| `begin` | Kích hoạt toàn bộ flow như cảm biến giấy, không phụ thuộc `paper=YES` |
| `motor:<0-100>` | Điều khiển motor load |
| `servo:<90-150>` | Điều khiển servo |
| `sensor` | Đọc cảm biến giấy |
| `limits` | Đọc sensor1/sensor2 |
| `status` | Đọc trạng thái hệ thống |
| `stop` | Dừng toàn bộ |
| `help` | Xem danh sách lệnh |

Mã lỗi Serial được ghi trong [errors.md](errors.md).
