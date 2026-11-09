# Serial Error Codes

Serial chỉ in mã lỗi dạng `eN`.

| Mã | Ý nghĩa |
|---|---|
| `e1` | Lệnh Serial không hợp lệ |
| `e2` | Sai cú pháp lệnh `push_t` |
| `e3` | Thời gian `push_t` ngoài giới hạn |
| `e4` | Không thể `begin`, hệ thống đang chờ `home` |
| `e5` | Không thể `begin`, flow đang bận |
| `e6` | Lệnh `home` bị chặn bởi công tắc hành trình |
| `e7` | Auto push bị chặn bởi công tắc hành trình |
| `e8` | Timeout hồi phục push |
| `e9` | Timeout re-approach, chưa chạm lại công tắc |
