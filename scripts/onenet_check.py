#!/usr/bin/env python3
"""OneNET 开放接口连通性自检.

用法:
    python3 scripts/onenet_check.py <product_id> <device_name> <access_key>

功能:
    按 App 里的同一套算法算 token, 调 query-device-property,
    打印平台返回的 code/msg, 用来区分"设备找不到" / "密钥不对" / "权限不够".

返回:
    退出码 0 = 配置可用; 非 0 = 失败 (原因见输出).

注意:
    access_key 是**设备级** AccessKey (Base64 字符串), 前后不要带空格.
"""

import base64
import hashlib
import hmac
import json
import sys
import urllib.parse
import urllib.request

API_BASE = 'https://iot-api.heclouds.com'
VERSION = '2018-10-31'
METHOD = 'sha1'
# 与 App / 固件一致的长效过期时间 (2030-12-30)
ET = 1924833600


def build_token(product_id, device_name, access_key):
    """按 OneNET 2018-10-31 协议算 token.

    参数:
        product_id:  产品 ID
        device_name: 设备名
        access_key:  设备级 AccessKey (Base64)
    返回:
        (token 字符串, 密钥问题提示或 None)
    """
    ak = access_key.strip()
    hint = None
    if ak != access_key:
        hint = 'AccessKey 前后有空白字符, 已在本次检查中忽略'
    try:
        key = base64.b64decode(ak, validate=True)
    except Exception:
        return '', f'AccessKey 不是合法 Base64 (长度 {len(ak)}), 多半是复制少了/多了字符'
    res = f'products/{product_id}/devices/{device_name}'
    src = f'{ET}\n{METHOD}\n{res}\n{VERSION}'.encode()
    sign = base64.b64encode(hmac.new(key, src, hashlib.sha1).digest()).decode()
    token = (f'version={urllib.parse.quote(VERSION, safe="")}'
             f'&res={urllib.parse.quote(res, safe="")}'
             f'&et={ET}'
             f'&method={METHOD}'
             f'&sign={urllib.parse.quote(sign, safe="")}')
    return token, hint


def main():
    if len(sys.argv) != 4:
        print(__doc__)
        return 2
    product_id, device_name, access_key = sys.argv[1], sys.argv[2], sys.argv[3]

    if access_key != access_key.strip():
        print(f'[警告] AccessKey 长度 {len(access_key)}, 前后有空白 ({len(access_key) - len(access_key.strip())} 个)')

    token, hint = build_token(product_id, device_name, access_key)
    if hint:
        print(f'[提示] {hint}')
    print(f'产品 ID: {product_id}  设备名: {device_name}  Key 长度: {len(access_key.strip())}')
    if not token:
        return 1

    q = urllib.parse.urlencode({'product_id': product_id, 'device_name': device_name})
    req = urllib.request.Request(f'{API_BASE}/thingmodel/query-device-property?{q}',
                                 headers={'Authorization': token})
    try:
        with urllib.request.urlopen(req, timeout=15) as r:
            body = json.loads(r.read().decode())
    except Exception as e:
        print(f'[失败] 请求异常: {e}')
        return 1

    print(f'[平台返回] code={body.get("code")} msg={body.get("msg")}')
    if body.get('code') == 0:
        print('[结论] 配置可用')
        return 0
    msg = str(body.get('msg', ''))
    if 'not found' in msg:
        print('[结论] 平台按这个产品 ID + 设备名找不到设备: 产品 ID 或设备名不对')
    elif 'invalid authorization' in msg:
        print('[结论] 设备存在但鉴权不过: AccessKey 不对 (不是这台设备的 key / 复制有误)')
    elif 'authPermissionDeny' in msg or body.get('code') == 10403:
        print('[结论] 鉴权被拒: 用错级别的 key 或该 key 无权访问此设备')
    return 1


if __name__ == '__main__':
    sys.exit(main())
