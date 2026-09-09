# Contributing

感谢参与 PICO × MANUS × Tactile Capture。

## 开发流程

1. 从 `main` 创建短生命周期分支。
2. 保持改动聚焦，并为故障修复补充回归测试。
3. 在 Windows + Python 3.11 上安装依赖并运行测试：

```powershell
python -m pip install -r requirements-windows.txt
python -m unittest discover -s tests -v
```

4. 确认 PowerShell 脚本可以被解析，并执行一键入口的 dry run：

```powershell
Get-ChildItem scripts -Filter *.ps1 | ForEach-Object {
  $tokens = $null; $errors = $null
  [System.Management.Automation.Language.Parser]::ParseFile(
    $_.FullName, [ref]$tokens, [ref]$errors
  ) | Out-Null
  if ($errors.Count) { throw "$($_.Name): $errors" }
}
powershell -ExecutionPolicy Bypass -File .\scripts\collect_windows.ps1 `
  -TaskPrefix dry_run -DryRun
```

## 数据与隐私

Pull request 中不得包含：

- `data/`、视频、JSONL、HDF5 或 review 产物；
- 个人 MANUS `.mcal`、真实 glove ID；
- PICO/触觉序列号和设备专属相机参数；
- 本机路径、内网 IP、日志、账号、密钥或访问令牌；
- MANUS SDK 的 DLL、头文件、库或厂商示例源码。

测试夹具必须使用虚构设备 ID、保留地址段和最小合成数据。

## 设计原则

- 数据完整性优先：错误必须 fail-closed，不能把不确定结果标记为成功。
- 采集线程不可被显示、解码或慢磁盘反向阻塞。
- STOP 必须有界，并验证文件与服务终态。
- 保持旧数据可读取；新增 schema 字段要记录版本与来源。
- Windows 是主要采集平台，Linux 脚本用于兼容和离线处理。
