/**
 * @file onenet_proxy.js
 * @brief OneNet HTTP API 开发代理
 *        - 解决浏览器 CORS 限制（拦截 OPTIONS 预检请求并返回 CORS 头）
 *        - 添加 Authorization 头后转发到 OneNet
 *
 * 运行方式: node onenet_proxy.js
 * 默认监听 7898 端口
 *
 * 代理地址: http://localhost:7898
 */

const http = require('http')
const https = require('https')

const PROXY_PORT = 7898
const ONENET_HOST = 'open.iot.10086.cn'

// 允许的来源
const ALLOWED_ORIGIN = 'http://127.0.0.1:8080'

const server = http.createServer((req, res) => {
  // CORS 预检请求
  if (req.method === 'OPTIONS') {
    res.writeHead(204, {
      'Access-Control-Allow-Origin': ALLOWED_ORIGIN,
      'Access-Control-Allow-Methods': 'GET, POST, OPTIONS',
      'Access-Control-Allow-Headers': 'Content-Type, Authorization, X-Device-AccessKey',
      'Access-Control-Max-Age': '86400'
    })
    res.end()
    return
  }

  // 从请求头获取 AccessKey（客户端从 localStorage 传过来，避免 dev 环境明文存文件）
  const accessKey = req.headers['x-device-accesskey'] || ''
  const authToken = req.headers['authorization'] || ''

  if (!authToken) {
    res.writeHead(400, { 'Content-Type': 'application/json' })
    res.end(JSON.stringify({ error: 'Missing Authorization header' }))
    return
  }

  const options = {
    hostname: ONENET_HOST,
    path: req.url,
    method: req.method,
    headers: {
      'Authorization': authToken,
      'Content-Type': 'application/json'
    }
  }

  const proxyReq = https.request(options, (proxyRes) => {
    res.writeHead(proxyRes.statusCode, {
      'Access-Control-Allow-Origin': ALLOWED_ORIGIN,
      'Access-Control-Allow-Methods': 'GET, POST, OPTIONS',
      'Access-Control-Allow-Headers': 'Content-Type, Authorization, X-Device-AccessKey',
      'Content-Type': 'application/json'
    })
    proxyRes.pipe(res, { end: true })
  })

  proxyReq.on('error', (err) => {
    res.writeHead(502, { 'Content-Type': 'application/json' })
    res.end(JSON.stringify({ error: err.message }))
  })

  if (req.method !== 'GET' && req.method !== 'HEAD') {
    req.pipe(proxyReq, { end: true })
  } else {
    proxyReq.end()
  }
})

server.listen(PROXY_PORT, () => {
  console.log(`OneNet proxy running at http://localhost:${PROXY_PORT}`)
  console.log(`Forwarding to https://${ONENET_HOST}`)
})
