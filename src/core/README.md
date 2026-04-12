# Core Layer

這一層保留所有 vehicle 共享的 runtime 能力。

預計內容：

- channel state storage
- timeout / signal validity
- status / LED state
- hardware ownership
- shared runtime context

目前已建立第一版檔案：

- `V7RCRuntimeState.h`
- `V7RCRuntimeState.cpp`

這個 runtime state 目前先提供：

- 16-channel state storage
- last frame timestamp
- timeout-based signal validity query
