# Release Checklist

這份清單用來做 `V7RCRobot-esp32` 發版前的最終確認。

## Metadata

- 確認 `library.properties` 的 `version` 已更新
- 確認 `library.properties` 的 `name`、`url`、`architectures`、`depends` 正確
- 確認 public include 入口仍為：
  - `V7RCRobot-esp32.h`
  - `V7RCCar-esp32.h`

## Documentation

- 確認 `README.md` 反映目前對外可用入口
- 確認 `QUICK_START_EDU.md` 的 example 路徑正確
- 確認 `docs/architecture/*` 仍與目前重構方向一致
- 確認是否需要更新 `keywords.txt`

## Validation

- 執行 `./scripts/compile-examples.sh`
- 執行 `ARDUINO_LINT=arduino-lint ./scripts/lint-library.sh`
- 若本機 `arduino-cli` 不在 `PATH`，先設定：
  - `ARDUINO_CLI=/path/to/arduino-cli`
- 若本機 `arduino-lint` 不在 `PATH`，先設定：
  - `ARDUINO_LINT=/path/to/arduino-lint`

## Examples

- 確認 legacy examples 可編譯：
  - `ESP32_C3_Differential_Drive`
  - `ESP32_C3_Mini_Mecanum_V3`
- 確認 clean examples 可編譯：
  - `ESP32_C3_Differential_Drive_Clean`
  - `ESP32_C3_Mini_Mecanum_Clean`

## Release Steps

1. 更新版本號
2. 更新 release notes / changelog
3. 建立 git tag
4. 建立 GitHub release
5. 若要送 Arduino Library Manager，確認使用的是要提交的 release/tag
