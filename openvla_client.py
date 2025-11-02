"""
OpenVLA远程推理客户端脚本

这个脚本演示如何调用OpenVLA的远程推理接口，包括图片格式转换和API调用。

依赖安装:
pip install requests json-numpy pillow numpy

使用方法:
python openvla_client.py --image_path your_image.jpg --instruction "your instruction" --server_url http://localhost:8000
"""

import argparse
import json
from pathlib import Path
from typing import Optional, Dict, Any
import time

import requests
import json_numpy
import numpy as np
from PIL import Image

# 启用json_numpy支持，用于处理numpy数组的序列化
json_numpy.patch()


class OpenVLAClient:
    """OpenVLA远程推理客户端"""
    
    def __init__(self, server_url: str = "http://localhost:8000"):
        self.server_url = server_url.rstrip('/')
        self.act_endpoint = f"{self.server_url}/act"
        # 请求相关的性能指标
        self.last_request_duration: Optional[float] = None
        self.last_inference_time: Optional[float] = None
        self.last_network_overhead: Optional[float] = None
    
    def preprocess_image(self, image_path: str, target_size: tuple = (256, 256)) -> np.ndarray:
        """
        预处理图片，转换为接口可接受的格式
        
        Args:
            image_path: 图片文件路径
            target_size: 目标尺寸，默认为(256, 256)
            
        Returns:
            numpy数组，形状为(height, width, 3)，数据类型为uint8
        """
        # 1. 使用PIL打开图片
        image = Image.open(image_path)
        
        # 2. 转换为RGB格式（处理RGBA、灰度图等情况）
        if image.mode != 'RGB':
            image = image.convert('RGB')
        
        # 3. 调整尺寸到目标大小
        image_resized = image.resize(target_size, Image.Resampling.LANCZOS)
        
        # 4. 转换为numpy数组，数据类型为uint8
        image_array = np.array(image_resized, dtype=np.uint8)
        
        print(f"图片预处理完成:")
        print(f"  原始尺寸: {image.size}")
        print(f"  目标尺寸: {target_size}")
        print(f"  数组形状: {image_array.shape}")
        print(f"  数据类型: {image_array.dtype}")
        print(f"  数值范围: [{image_array.min()}, {image_array.max()}]")
        
        return image_array
    
    def send_request(self, image_array: np.ndarray, instruction: str, 
                    unnorm_key: Optional[str] = None) -> Dict[str, Any]:
        """
        发送推理请求到OpenVLA服务器
        
        Args:
            image_array: 预处理后的图片数组
            instruction: 指令文本
            unnorm_key: 可选的反归一化键
            
        Returns:
            服务器返回的响应字典
        """
        # 构建请求数据
        payload = {
            "image": image_array,
            "instruction": instruction
        }
        
        if unnorm_key:
            payload["unnorm_key"] = unnorm_key
        
        try:
            request_start = time.time()
            # 重置指标
            self.last_request_duration = None
            self.last_inference_time = None
            self.last_network_overhead = None

            print(f"\n发送请求到: {self.act_endpoint}")
            print(f"指令: {instruction}")
            if unnorm_key:
                print(f"反归一化键: {unnorm_key}")

            # 发送POST请求
            response = requests.post(
                self.act_endpoint,
                json=payload,
                timeout=30  # 30秒超时
            )

            self.last_request_duration = time.time() - request_start

            # 检查响应状态
            if response.status_code == 200:
                result = response.json()
                print("推理成功!")
                inference_time = result.get("inference_time_seconds")
                if inference_time is not None:
                    try:
                        self.last_inference_time = float(inference_time)
                        network_overhead = self.last_request_duration - self.last_inference_time
                        if network_overhead < 0:
                            network_overhead = 0.0
                        self.last_network_overhead = network_overhead
                        print(f"推理耗时: {self.last_inference_time:.3f} 秒")
                        print(f"HTTP往返耗时: {self.last_request_duration:.3f} 秒")
                        print(f"网络传输耗时(估计): {self.last_network_overhead:.3f} 秒")
                    except (TypeError, ValueError):
                        self.last_inference_time = None
                        self.last_network_overhead = None
                        print("推理耗时字段无法解析")
                else:
                    if self.last_request_duration is not None:
                        print(f"HTTP往返耗时: {self.last_request_duration:.3f} 秒")
                return result
            else:
                print(f"请求失败，状态码: {response.status_code}")
                print(f"错误信息: {response.text}")
                return {"error": f"HTTP {response.status_code}", "message": response.text}
                
        except requests.exceptions.ConnectionError:
            error_msg = f"无法连接到服务器: {self.server_url}"
            print(error_msg)
            return {"error": "ConnectionError", "message": error_msg}
        except requests.exceptions.Timeout:
            error_msg = "请求超时"
            print(error_msg)
            return {"error": "Timeout", "message": error_msg}
        except Exception as e:
            error_msg = f"请求异常: {str(e)}"
            print(error_msg)
            return {"error": "Exception", "message": error_msg}  
    
    def parse_numpy_result(self, result_data: Any) -> np.ndarray:
        """
        解析numpy格式的推理结果
        
        Args:
            result_data: 推理结果数据，可能是base64编码的字符串或字典
            
        Returns:
            解析后的numpy数组
        """
        print("inferrence result:  ",result_data)
        try:
            # 如果结果是字典，包含标准的numpy序列化格式
            if isinstance(result_data, dict):
                # 检查是否包含标准的numpy序列化字段
                if "__numpy__" in result_data and "dtype" in result_data and "shape" in result_data:
                    base64_str = result_data["__numpy__"]
                    dtype_str = result_data["dtype"]
                    shape = result_data["shape"]
                    
                    print(f"解析标准numpy格式: dtype={dtype_str}, shape={shape}")
                    
                    # 解码base64数据
                    import base64
                    decoded_bytes = base64.b64decode(base64_str)
                    
                    # 根据dtype字符串确定数据类型
                    dtype_map = {
                        "<f8": np.float64,
                        "<f4": np.float32,
                        "<i8": np.int64,
                        "<i4": np.int32
                    }
                    
                    dtype = dtype_map.get(dtype_str, np.float64)
                    
                    # 使用正确的数据类型解析
                    array = np.frombuffer(decoded_bytes, dtype=dtype)
                    
                    # 如果形状不匹配，尝试reshape
                    if len(array) != shape[0]:
                        print(f"警告: 数组长度{len(array)}与形状{shape}不匹配，尝试reshape")
                        try:
                            array = array.reshape(shape)
                        except:
                            print(f"reshape失败，返回原始数组")
                    
                    print(f"✓ 成功解析numpy数组: shape={array.shape}, dtype={array.dtype}")
                    return array
                
                # 查找包含numpy数据的其他键
                for key, value in result_data.items():
                    if isinstance(value, str) and value.startswith("__numpy__"):
                        return self.parse_numpy_result(value)
                
                # 如果字典中有直接的数组数据
                if "action" in result_data:
                    action_data = result_data["action"]
                    if isinstance(action_data, np.ndarray):
                        if action_data.shape == (7,) or len(action_data) == 7:
                            return action_data.astype(np.float32)
                    if isinstance(action_data, list) and len(action_data) == 7:
                        return np.array(action_data, dtype=np.float32)
                    if isinstance(action_data, tuple) and len(action_data) == 7:
                        return np.array(action_data, dtype=np.float32)
            
            # 如果结果是字符串，尝试解析base64编码的numpy数组
            elif isinstance(result_data, str):
                # 检查是否是base64编码的numpy数组格式
                if result_data.startswith("__numpy__"):
                    # 提取base64部分
                    base64_str = result_data.replace("__numpy__", "")
                    # 使用json_numpy解析
                    import base64
                    decoded_bytes = base64.b64decode(base64_str)
                    
                    # 尝试不同的数据类型来解析
                    data_types = [np.float64, np.float32, np.int64, np.int32]
                    
                    for dtype in data_types:
                        try:
                            array = np.frombuffer(decoded_bytes, dtype=dtype)
                            if len(array) == 7:
                                print(f"✓ 使用 {dtype} 成功解析7元素数组")
                                return array
                        except:
                            continue
                    
                    # 如果所有类型都无法解析为7个元素，使用float32并取前7个
                    array = np.frombuffer(decoded_bytes, dtype=np.float32)
                    print(f"警告: 无法直接解析为7元素数组，使用float32并取前7个元素")
                    return array[:7]
            
            # 如果已经是numpy数组
            elif isinstance(result_data, np.ndarray):
                if result_data.shape == (7,) or (len(result_data) == 7):
                    return result_data.astype(np.float32)
            
            # 如果是列表
            elif isinstance(result_data, list) and len(result_data) == 7:
                return np.array(result_data, dtype=np.float32)
                
        except Exception as e:
            print(f"解析numpy结果时出错: {e}")
        
        # 如果无法解析，返回默认的7维零数组
        print("无法解析推理结果，返回默认零数组")
        return np.zeros(7, dtype=np.float32)

    def predict_action(self, image_path: str, instruction: str, 
                      unnorm_key: Optional[str] = None) -> Dict[str, Any]:
        """
        完整的推理流程：预处理图片 + 发送请求
        
        Args:
            image_path: 图片文件路径
            instruction: 指令文本
            unnorm_key: 可选的反归一化键
            
        Returns:
            推理结果
        """
        # 1. 预处理图片
        image_array = self.preprocess_image(image_path)
        
        # 2. 发送推理请求
        result = self.send_request(image_array, instruction, unnorm_key)
        
        # 3. 解析推理结果
        print(f"原始推理结果类型: {type(result)}")
        print(f"原始推理结果内容: {result}")
        
        # 如果结果是numpy数组，直接使用
        if isinstance(result, np.ndarray):
            print("✓ 推理结果是numpy数组，直接使用")
            parsed_result = {
                "original_array": result.tolist(),
                "parsed_action": result.tolist(),
                "action_shape": result.shape,
                "action_dtype": str(result.dtype)
            }
            return parsed_result
        
        # 如果结果是字典，尝试解析
        elif isinstance(result, dict) and "error" not in result:
            # 创建新的结果字典，避免修改只读的原始结果
            parsed_result = result.copy() if hasattr(result, 'copy') else dict(result)
            parsed_action = self.parse_numpy_result(result)
            parsed_result["parsed_action"] = parsed_action.tolist()
            parsed_result["action_shape"] = parsed_action.shape
            parsed_result["action_dtype"] = str(parsed_action.dtype)
            return parsed_result
        
        return result


def main():
    """主函数，处理命令行参数并执行推理"""
    parser = argparse.ArgumentParser(description="OpenVLA远程推理客户端")
    parser.add_argument("--image_path", type=str, required=True, 
                       help="输入图片路径")
    parser.add_argument("--instruction", type=str, required=True,
                       help="指令文本")
    parser.add_argument("--server_url", type=str, default="http://localhost:8000",
                       help="服务器URL，默认: http://localhost:8000")
    parser.add_argument("--unnorm_key", type=str, default=None,
                       help="可选的反归一化键")
    parser.add_argument("--target_size", type=str, default="256,256",
                       help="目标图片尺寸，格式: width,height，默认: 256,256")
    
    args = parser.parse_args()
    
    # 解析目标尺寸
    try:
        width, height = map(int, args.target_size.split(','))
        target_size = (width, height)
    except ValueError:
        print("错误: 目标尺寸格式不正确，使用默认尺寸256x256")
        target_size = (256, 256)
    
    # 创建客户端
    client = OpenVLAClient(args.server_url)
    
    # 执行推理
    result = client.predict_action(args.image_path, args.instruction, args.unnorm_key)
    
    # 输出结果
    print("\n" + "="*50)
    print("推理结果:")
    print("="*50)
    print(json.dumps(result, indent=2, ensure_ascii=False))


def example_usage():
    """使用示例"""
    print("="*60)
    print("OpenVLA客户端使用示例")
    print("="*60)
    
    # 示例1: 基本用法
    print("\n1. 基本用法:")
    print("python openvla_client.py --image_path test.jpg --instruction \"拿起杯子\"")
    
    # 示例2: 指定服务器地址
    print("\n2. 指定服务器地址:")
    print("python openvla_client.py --image_path test.jpg --instruction \"拿起杯子\" --server_url http://192.168.1.100:8000")
    # python openvla_client.py --image_path cup.png --instruction \"拿起杯子\" --server_url http://100.64.51.46:8000
    
    # 示例3: 使用反归一化键
    print("\n3. 使用反归一化键:")
    print("python openvla_client.py --image_path test.jpg --instruction \"拿起杯子\" --unnorm_key \"calvin\"")
    
    # 示例4: 自定义图片尺寸
    print("\n4. 自定义图片尺寸:")
    print("python openvla_client.py --image_path test.jpg --instruction \"拿起杯子\" --target_size 224,224")
    
    print("\n" + "="*60)


if __name__ == "__main__":
    # 显示使用示例
    example_usage()
    
    # 如果提供了命令行参数，则执行主函数
    import sys
    if len(sys.argv) > 1:
        main()
    else:
        print("\n请使用命令行参数运行脚本，或查看上面的使用示例。")
