# GitHub 配置

## 仓库信息

- 仓库: https://github.com/mack-hx/esp-
- HTTPS: https://github.com/mack-hx/esp-.git
- SSH: git@github.com:mack-hx/esp-.git (可能被代理拦截, 优先 HTTPS)

## 认证

凭据从系统钥匙串取, 不落盘、不进仓库:

```bash
export GH_TOKEN=$(git credential-osxkeychain get <<'EOF' 2>/dev/null | sed -n 's/^password=//p'
protocol=https
host=github.com

EOF
)
```

## 推送

正常情况: 全局 git 代理指向 `http://127.0.0.1:7890`, 直接推送即可:

```bash
git push origin main
```

## 备用方案 (直连被重置时)

本机 DNS 把 `github.com` 解析到亚洲 IP, 直连时 HTTP/2 会被 `Connection reset by peer`
重置, 表现为 "Operation too slow"。`api.github.com` 直连正常, 只有 `github.com` 会卡。

```bash
GIT_TERMINAL_PROMPT=0 GIT_CONFIG_COUNT=3 \
GIT_CONFIG_KEY_0=http.proxy GIT_CONFIG_VALUE_0= \
GIT_CONFIG_KEY_1=https.proxy GIT_CONFIG_VALUE_1= \
GIT_CONFIG_KEY_2=http.version GIT_CONFIG_VALUE_2=HTTP/1.1 \
GIT_HTTP_LOW_SPEED_LIMIT=1 GIT_HTTP_LOW_SPEED_TIME=120 \
git push origin main
```

- `GIT_CONFIG_COUNT` / `GIT_CONFIG_KEY_n` / `GIT_CONFIG_VALUE_n` 用环境变量临时清空
  http.proxy / https.proxy, 不改全局 git config
- `http.version=HTTP/1.1` 是关键, 去掉会卡在低速超时
- `GIT_HTTP_LOW_SPEED_LIMIT=1` + `GIT_HTTP_LOW_SPEED_TIME=120` 放宽低速判定, 避免慢速被误杀
- 推 tag 用同一条命令 (把 `main` 换成 tag 名); `gh release create` 也建议带同样的前缀
