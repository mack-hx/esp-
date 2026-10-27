/**
 * @file vite.config.js
 * @brief Vite 开发服务器配置
 *        - 代理 /onenet-api/* 请求到本地 OneNet 代理（解决 CORS）
 *        - H5 devServer: http://127.0.0.1:8080
 */

import { defineConfig } from 'vite'
import uni from '@dcloudio/vite-plugin-uni'

export default defineConfig({
  plugins: [uni()],
  server: {
    port: 8080,
    proxy: {
      // 代理 /onenet-api/* 到本地 OneNet 代理服务
      '/onenet-api': {
        target: 'http://localhost:7898',
        changeOrigin: true,
        rewrite: (path) => path.replace(/^\/onenet-api/, '')
      }
    }
  }
})
