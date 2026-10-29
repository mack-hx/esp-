# H5 调试规则

## 前置条件
- **必须先启动 H5**：用户在 HBuilderX 中运行 → 运行到浏览器 → Chrome
- 服务器地址：`http://127.0.0.1:8080/`
- **必须先确认服务器运行**：用 `lsof -i :8080` 检查端口是否有 node 进程监听

## 调试流程

### 1. 启动调试
```
用户操作：HBuilderX → 运行 → 运行到浏览器 → Chrome
```

### 2. 连接浏览器
```javascript
// 连接到 H5 页面
browser_navigate({ url: "http://127.0.0.1:8080/" })

// 如果已有关联的标签页，直接使用
browser_snapshot()  // 获取当前页面元素
browser_take_screenshot()  // 截图查看效果
```

### 3. 修改代码
- 助手修改 `.vue` 文件后保存
- HBuilderX 会自动编译并热更新（不需要手动重启）

### 4. 刷新页面验证
```javascript
// 方式1: 重新导航（强制刷新）
browser_navigate({ url: "http://127.0.0.1:8080/" })

// 方式2: CDP 刷新
browser_cdp({ method: "Page.reload" })

// 方式3: 浏览器按 F5 或 Cmd+R（但助手无法模拟）
```

### 5. 验证效果
```javascript
browser_snapshot()  // 获取新布局的元素结构
browser_take_screenshot()  // 截图确认视觉变化
```

### 检查服务器状态
```bash
lsof -i :8080
```
- 有 node 进程监听 = 服务器正常
- 无输出 = 服务器未启动

### 缓存清理
如果修改没生效，在 Chrome 中：
- Mac: `Cmd+Shift+R` 强制刷新
- Windows: `Ctrl+Shift+R`

## 调试示例

### 场景：修改 DPad 布局
1. 修改 `control.vue` 文件
2. 确认服务器运行：`lsof -i :8080`
3. 刷新页面：`browser_navigate({ url: "http://127.0.0.1:8080/" })`
4. 验证：`browser_take_screenshot()`
