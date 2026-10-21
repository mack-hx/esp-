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

## 注意事项
- SSH方式可能被代理拦截(GitHub被解析到127.0.0.1:7897)
- 优先使用HTTPS + Token方式
