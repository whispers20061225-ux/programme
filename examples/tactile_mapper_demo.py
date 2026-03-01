# examples/tactile_mapper_demo.py
"""
触觉映射器使用示例
"""

import sys
import os
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
src_root = os.path.join(project_root, "src")
if src_root not in sys.path:
    sys.path.insert(0, src_root)
if project_root not in sys.path:
    sys.path.insert(0, project_root)

import numpy as np
import matplotlib.pyplot as plt
import json
import time
from tactile_perception import TactileMapper

def demo_basic_mapping():
    """基础映射演示"""
    print("=== 基础触觉映射演示 ===")
    
    # 创建映射器
    mapper = TactileMapper(grid_shape=(3, 3), sensor_size=(20.0, 20.0))
    
    # 创建模拟数据
    forces = np.zeros((9, 3))
    
    # 设置不同测点的力值
    forces[4, 2] = 5.0  # 中心点
    forces[1, 2] = 2.0  # 左上
    forces[3, 2] = 2.0  # 左中
    forces[5, 2] = 2.0  # 右中
    forces[7, 2] = 2.0  # 下中
    
    print(f"原始力数据形状: {forces.shape}")
    print(f"测点位置: {mapper.tactile_positions}")
    
    # 创建触觉图像
    tactile_image = mapper.create_tactile_image(forces, force_component=2)
    print(f"触觉图像形状: {tactile_image.shape}")
    
    # 计算统计信息
    stats = mapper.get_contact_statistics(tactile_image)
    print(f"\n接触统计信息:")
    print(f"  接触面积: {stats['contact_area_mm2']:.2f} mm²")
    print(f"  力质心: ({stats['centroid_x_mm']:.2f}, {stats['centroid_y_mm']:.2f}) mm")
    print(f"  总力: {stats['total_force_n']:.2f} N")
    print(f"  最大力: {stats['max_force_n']:.2f} N")
    print(f"  接触比例: {stats['contact_ratio']:.2%}")
    
    # 可视化
    mapper.visualize_tactile_image(tactile_image, title="基础触觉映射演示")
    
    return mapper, forces, tactile_image

def demo_advanced_features():
    """高级功能演示"""
    print("\n=== 高级功能演示 ===")
    
    mapper = TactileMapper()
    
    # 创建更复杂的模拟数据
    np.random.seed(42)
    forces = np.random.rand(9, 3) * 3
    forces[:, 2] = np.abs(forces[:, 2])  # Fz为正
    
    # 1. 创建力向量场
    grid_x, grid_y, fx, fy = mapper.create_force_vector_field(forces)
    print(f"力向量场形状: {fx.shape}")
    
    # 2. 计算散度和旋度
    divergence = mapper.calculate_force_divergence(fx, fy)
    curl = mapper.calculate_force_curl(fx, fy)
    print(f"散度范围: [{divergence.min():.3f}, {divergence.max():.3f}]")
    print(f"旋度范围: [{curl.min():.3f}, {curl.max():.3f}]")
    
    # 3. 分割接触区域
    tactile_image = mapper.create_tactile_image(forces)
    regions = mapper.segment_contact_regions(tactile_image, threshold=0.5)
    print(f"检测到 {len(regions)} 个接触区域")
    
    for i, region in enumerate(regions):
        print(f"  区域 {i+1}: 面积={region['area']:.2f} mm², "
              f"平均力={region['mean_intensity']:.2f} N")
    
    # 4. 创建力分布直方图
    hist_data = mapper.create_force_distribution_histogram(tactile_image)
    print(f"\n力分布统计:")
    print(f"  样本数: {hist_data['num_samples']}")
    print(f"  平均力: {hist_data['mean']:.3f} N")
    print(f"  力标准差: {hist_data['std']:.3f} N")
    print(f"  偏度: {hist_data['skewness']:.3f}")
    print(f"  峰度: {hist_data['kurtosis']:.3f}")
    
    # 5. 创建热图叠加层
    heatmap = mapper.create_contact_heatmap_overlay(tactile_image, alpha=0.6)
    print(f"热图叠加层形状: {heatmap.shape}")
    
    # 可视化力分布直方图
    plt.figure(figsize=(10, 4))
    
    plt.subplot(121)
    plt.bar(range(len(hist_data['hist_values'])), hist_data['hist_values'])
    plt.title('力分布直方图')
    plt.xlabel('力区间')
    plt.ylabel('频数')
    
    plt.subplot(122)
    plt.imshow(heatmap, origin='lower', extent=[-10, 10, -10, 10])
    plt.title('接触热图叠加')
    plt.xlabel('X 位置 (mm)')
    plt.ylabel('Y 位置 (mm)')
    plt.colorbar(label='力强度')
    
    plt.tight_layout()
    plt.show()
    
    return mapper, forces, regions

def demo_export_import():
    """数据导出导入演示"""
    print("\n=== 数据导出导入演示 ===")
    
    mapper = TactileMapper()
    
    # 创建模拟数据
    forces = np.array([
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 2.0],
        [0.0, 0.0, 1.5],
        [0.0, 0.0, 0.8],
        [0.0, 0.0, 4.0],
        [0.0, 0.0, 0.9],
        [0.0, 0.0, 1.2],
        [0.0, 0.0, 0.7],
        [0.0, 0.0, 1.8]
    ])
    
    # 创建触觉图像
    tactile_image = mapper.create_tactile_image(forces)
    
    # 导出数据
    metadata = {
        'description': '触觉数据演示',
        'timestamp': time.time(),
        'sensor_type': 'MC-M2020',
        'units': {'force': 'N', 'distance': 'mm'}
    }
    
    exported_data = mapper.export_tactile_data(tactile_image, metadata=metadata)
    
    # 保存到文件
    import tempfile
    temp_file = tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False)
    json.dump(exported_data, temp_file, indent=2)
    temp_file.close()
    
    print(f"数据已保存到: {temp_file.name}")
    print(f"导出数据大小: {len(json.dumps(exported_data))} 字节")
    print(f"包含的键: {list(exported_data.keys())}")
    
    # 从文件加载
    with open(temp_file.name, 'r') as f:
        loaded_data = json.load(f)
    
    # 导入数据
    imported_image = mapper.load_tactile_data(loaded_data)
    
    if imported_image is not None:
        print(f"成功导入图像，形状: {imported_image.shape}")
        
        # 验证数据一致性
        if np.allclose(tactile_image, imported_image, rtol=1e-5):
            print("导入数据与原始数据一致 ✓")
        else:
            print("警告: 导入数据与原始数据存在差异")
        
        # 显示元数据
        if 'metadata' in loaded_data:
            print(f"元数据: {loaded_data['metadata']}")
    
    # 清理临时文件
    os.unlink(temp_file.name)
    
    return exported_data, imported_image

def demo_real_time_simulation():
    """实时模拟演示"""
    print("\n=== 实时模拟演示 ===")
    
    mapper = TactileMapper()
    
    # 创建动画
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    plt.ion()  # 交互模式
    
    # 模拟实时数据
    for frame in range(50):
        # 生成动态数据
        forces = np.random.randn(9, 3) * 0.5
        forces[:, 2] = np.abs(forces[:, 2]) + 1.0  # 确保Fz为正
        
        # 添加周期性变化
        forces[:, 2] *= (1 + 0.5 * np.sin(frame * 0.2))
        
        # 创建触觉图像
        tactile_image = mapper.create_tactile_image(forces)
        
        # 清空轴
        for ax in axes:
            ax.clear()
        
        # 1. 2D热图
        im = axes[0].imshow(tactile_image.T, 
                           extent=[-10, 10, -10, 10],
                           origin='lower',
                           cmap='viridis',
                           vmin=0, vmax=5)
        axes[0].set_title(f'帧 {frame}: 触觉热图')
        axes[0].set_xlabel('X 位置 (mm)')
        axes[0].set_ylabel('Y 位置 (mm)')
        plt.colorbar(im, ax=axes[0], label='力 (N)')
        
        # 显示测点
        axes[0].scatter(mapper.tactile_positions[:, 0],
                       mapper.tactile_positions[:, 1],
                       c='red', s=30, marker='o')
        
        # 2. 3D表面图
        X, Y = np.meshgrid(np.linspace(-10, 10, tactile_image.shape[1]),
                          np.linspace(-10, 10, tactile_image.shape[0]))
        
        ax_3d = fig.add_subplot(122, projection='3d')
        surf = ax_3d.plot_surface(X, Y, tactile_image,
                                 cmap='viridis', alpha=0.8)
        ax_3d.set_title(f'帧 {frame}: 3D表面')
        ax_3d.set_xlabel('X (mm)')
        ax_3d.set_ylabel('Y (mm)')
        ax_3d.set_zlabel('力 (N)')
        ax_3d.set_zlim(0, 5)
        
        # 更新图形
        plt.draw()
        plt.pause(0.05)
        
        # 计算并显示统计信息
        if frame % 10 == 0:
            stats = mapper.get_contact_statistics(tactile_image)
            print(f"帧 {frame}: 总力={stats['total_force_n']:.2f}N, "
                  f"面积={stats['contact_area_mm2']:.2f}mm²")
    
    plt.ioff()
    plt.show()
    
    print("实时模拟完成")

def demo_custom_configuration():
    """自定义配置演示"""
    print("\n=== 自定义配置演示 ===")
    
    # 1. 不同传感器尺寸
    print("1. 不同传感器尺寸:")
    
    sizes = [(15, 15), (20, 20), (25, 25), (30, 20)]
    for size in sizes:
        mapper = TactileMapper(grid_shape=(3, 3), sensor_size=size)
        print(f"  传感器尺寸 {size}mm: 测点位置范围 "
              f"[{mapper.tactile_positions[:,0].min():.1f}, "
              f"{mapper.tactile_positions[:,0].max():.1f}]mm")
    
    # 2. 不同插值方法
    print("\n2. 不同插值方法:")
    
    methods = ['linear', 'cubic', 'nearest', 'rbf']
    forces = np.random.rand(9, 3) * 2
    forces[:, 2] = np.abs(forces[:, 2])
    
    for method in methods:
        try:
            mapper = TactileMapper(interpolation_method=method)
            image = mapper.create_tactile_image(forces)
            stats = mapper.get_contact_statistics(image)
            print(f"  方法 '{method}': 总力={stats['total_force_n']:.3f}N, "
                  f"平滑度={stats['force_std_n']:.3f}")
        except Exception as e:
            print(f"  方法 '{method}' 失败: {e}")
    
    # 3. 不同网格分辨率
    print("\n3. 不同网格分辨率:")
    
    mapper = TactileMapper()
    
    # 测试不同力分量
    for component in range(3):
        component_names = ['Fx', 'Fy', 'Fz']
        image = mapper.create_tactile_image(forces, force_component=component)
        stats = mapper.get_contact_statistics(image)
        
        print(f"  分量 {component_names[component]}: "
              f"总力={stats['total_force_n']:.3f}N, "
              f"最大力={stats['max_force_n']:.3f}N")

if __name__ == "__main__":
    print("触觉映射器演示程序")
    print("=" * 50)
    
    # 运行演示
    demo_basic_mapping()
    
    demo_advanced_features()
    
    demo_export_import()
    
    # 注释掉实时演示（需要交互）
    # demo_real_time_simulation()
    
    demo_custom_configuration()
    
    print("\n" + "=" * 50)
    print("所有演示完成")
