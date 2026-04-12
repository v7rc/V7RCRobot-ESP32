# Protocol Layer

這一層負責封包解析、格式驗證與統一 command/frame model。

第一階段將從 legacy driver 中拆出：

- `HEX`
- `DEG`
- `SRV / SRT`
- `SS8`
- `LED`

目前已建立第一版檔案：

- `V7RCProtocol.h`
- `V7RCProtocol.cpp`

目前 protocol layer 已負責：

- protocol type 判斷
- `HEX` / `DEG` / `SRV` / `SRT` / `SS8` / `LED` decode
- 統一 `V7RC_Frame` 結構
- channel presence 標記

legacy driver 目前仍保留與 servo init/default 值有關的 legacy fallback，後續可再逐步往 core runtime 整理。
