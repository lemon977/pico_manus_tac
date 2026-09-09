# Security and privacy

## Supported version

安全与隐私修复只针对 `main` 最新版本。

## Reporting

请通过仓库的 **Security → Report a vulnerability** 私下报告安全问题，不要在公开
issue 中粘贴密钥、设备序列号、人体数据、内网地址或完整日志。

报告中请仅提供复现所需的最小信息。提交前应移除：

- GitHub/SSH/API 凭据；
- PICO、MANUS 和触觉设备身份；
- 人体骨架、视频、触觉原始数据；
- 用户名、本机绝对路径和网络拓扑。

## Repository policy

仓库通过 `.gitignore` 排除采集数据、个人标定、设备标定、运行日志和第三方 MANUS
SDK。忽略规则不是安全边界；每次发布仍必须对 Git 暂存区执行独立敏感信息扫描。
