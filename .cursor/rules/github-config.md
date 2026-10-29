# GitHub 配置

## 仓库信息
- **仓库地址**: https://github.com/mack-hx/esp-
- **SSH地址**: git@github.com:mack-hx/esp-.git
- **HTTPS地址**: https://github.com/mack-hx/esp-.git

## 认证方式
- **推荐方式**: HTTPS + Token
- **Personal Access Token**: `YOUR_TOKEN_HERE`
- Token权限: repo (完整仓库访问)

## 使用方法
```bash
export GH_TOKEN="YOUR_TOKEN_HERE"
export PATH="/opt/homebrew/bin:$PATH"
gh release list --repo mack-hx/esp-
```

从系统钥匙串取 token (不落盘, 不进仓库):
```bash
export GH_TOKEN=$(git credential-osxkeychain get <<'EOF' 2>/dev/null | sed -n 's/^password=//p'
protocol=https
host=github.com

EOF
)
```

## 注意事项
- SSH方式可能被代理拦截(GitHub被解析到127.0.0.1:7890)
- 优先使用HTTPS + Token方式

## 推送命令

**正常情况（2026-10-27 起）**：全局 git 代理已指向 `http://127.0.0.1:7890`，
直接 `git push origin main` 就行，实测走代理 HTTP/2 正常，不需要任何额外参数：

```bash
git push origin main
```

## 备用方案（直连被重置时）

本机 DNS 把 `github.com` 解析到亚洲 IP (20.205.243.166), 直连时 HTTP/2 请求会被
`Connection reset by peer` 重置, git 表现为 "Operation too slow. Less than 1 bytes/sec
transferred the last 60 seconds"。`api.github.com` 直连正常, 只有 `github.com` 会卡。

解决: 强制 HTTP/1.1, 同时清掉代理。

```bash
GIT_TERMINAL_PROMPT=0 GIT_CONFIG_COUNT=3 \
GIT_CONFIG_KEY_0=http.proxy GIT_CONFIG_VALUE_0= \
GIT_CONFIG_KEY_1=https.proxy GIT_CONFIG_VALUE_1= \
GIT_CONFIG_KEY_2=http.version GIT_CONFIG_VALUE_2=HTTP/1.1 \
GIT_HTTP_LOW_SPEED_LIMIT=1 GIT_HTTP_LOW_SPEED_TIME=120 \
git push origin main
```

- `GIT_CONFIG_COUNT` / `GIT_CONFIG_KEY_n` / `GIT_CONFIG_VALUE_n` 是 git 的**环境变量**形式
  传配置, 用来临时清空 http.proxy / https.proxy, 不用改全局 git config
- `http.version=HTTP/1.1` 是关键: 去掉它会卡在低速超时
- `GIT_HTTP_LOW_SPEED_LIMIT=1` + `GIT_HTTP_LOW_SPEED_TIME=120`: 把"低速判定"放宽到 120 秒,
  避免慢速连接被误杀
- 打 tag 推送用同一条命令 (把 `main` 换成 tag 名)
- `gh release create` 也建议带上同样的 `GIT_CONFIG_*` 前缀, 否则同样会卡
