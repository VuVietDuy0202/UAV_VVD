# CLAUDE.md — UAV_VVD

## ⚠️ RULE QUAN TRỌNG NHẤT — KHÔNG SỬA CODE

Toàn bộ source code của project này là **ĐÚNG và ĐÃ ĐƯỢC KIỂM CHỨNG** (clone từ git,
đã từng chạy tốt). Khi có lỗi runtime/biên dịch, **nguyên nhân KHÔNG bao giờ nằm ở code**
mà ở **phiên bản platform/toolchain đã cài**.

**Vì vậy, TUYỆT ĐỐI KHÔNG:**
- Sửa, refactor, "cải thiện" bất kỳ file `.cpp` / `.h` nào trong `src/`, `include/`, `test/`.
- Đổi API để "cho hợp với core mới".
- Thay đổi logic để né lỗi biên dịch.

**Cách xử lý lỗi ĐÚNG:** chỉ điều chỉnh **phiên bản trong `platformio.ini`** cho khớp với
phiên bản mà code được viết. Không được đụng vào code.

## Nguyên nhân đã xác định (2026-07)

- `espressif32 @ 7.0.x` dùng **Arduino-ESP32 core 3.x** → thay đổi API (LEDC/Servo,
  analogWrite, Serial/USB CDC, WiFi...) → code core-2.x chạy sai dù vẫn biên dịch được.
- Toàn bộ dòng `espressif32 @ 6.x` (6.0.0 → 6.13.0) dùng **core 2.x** → tương thích code.

## Version đã ghim (BẮT BUỘC giữ)

```ini
platform = espressif32@6.9.0   ; core 2.x — KHÔNG đổi sang 7.x
```

Có thể dùng bất kỳ bản `6.x` nào (6.5.0 / 6.9.0 / 6.13.0) nếu cần, nhưng
**không được để `platform = espressif32` không ghim version** (sẽ tự kéo bản mới nhất).

## Sau khi đổi version — cần làm sạch build

```powershell
pio pkg install       # kéo đúng platform 6.9.0
pio run -t clean      # xoá build cũ của core 3.x
pio run               # build lại
```

Đường dẫn pio (không có trong PATH): `%USERPROFILE%\.platformio\penv\Scripts\pio.exe`
