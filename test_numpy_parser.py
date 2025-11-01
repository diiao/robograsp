#!/usr/bin/env python3
"""
测试numpy数组解析功能
"""

import base64
import numpy as np
from openvla_client import OpenVLAClient

def test_numpy_parsing():
    """测试numpy数组解析功能"""
    
    # 创建客户端实例
    client = OpenVLAClient()
    
    # 测试数据：你提供的base64编码字符串
    numpy_str = "__numpy__GEVPJlbtdb8AdI7qOU8av70hiA78So6/tHk8Ml/mnz8A+72zyCJUP24Enhc877C/AAAAAAAAAAA="
    
    print("测试numpy数组解析:")
    print("="*50)
    print(f"原始字符串: {numpy_str}")
    
    # 解析numpy数组
    parsed_array = client.parse_numpy_result(numpy_str)
    
    print(f"解析后的数组: {parsed_array}")
    print(f"数组形状: {parsed_array.shape}")
    print(f"数组类型: {parsed_array.dtype}")
    print(f"数组值: {parsed_array.tolist()}")
    
    # 验证数组长度是否为7
    if len(parsed_array) == 7:
        print("✓ 数组长度正确 (7个元素)")
    else:
        print(f"✗ 数组长度不正确: {len(parsed_array)} (期望: 7)")
    
    # 手动解析验证
    print("\n手动解析验证:")
    print("-"*30)
    
    # 提取base64部分
    base64_part = numpy_str.replace("__numpy__", "")
    print(f"Base64部分: {base64_part}")
    
    # 解码base64
    decoded_bytes = base64.b64decode(base64_part)
    print(f"解码后字节长度: {len(decoded_bytes)}")
    
    # 尝试不同的数据类型解析
    data_types = [np.float32, np.float64, np.int32, np.int64]
    
    for dtype in data_types:
        try:
            array = np.frombuffer(decoded_bytes, dtype=dtype)
            print(f"\n尝试 {dtype} 类型:")
            print(f"  数组长度: {len(array)}")
            print(f"  数组值: {array.tolist()}")
            
            if len(array) == 7:
                print("  ✓ 找到正确的7元素数组!")
                break
        except Exception as e:
            print(f"  ✗ 解析失败: {e}")

if __name__ == "__main__":
    test_numpy_parsing()