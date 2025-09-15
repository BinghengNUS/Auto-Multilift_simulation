#!/bin/bash

# MPC性能评估启动脚本
# MPC Performance Evaluation Launcher Script
# 作者: Assistant
# 日期: 2025-09-15

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 打印标题
print_title() {
    echo "============================================================"
    echo "🚀 MPC性能评估系统启动器"
    echo "🚀 MPC Performance Evaluation System Launcher"
    echo "============================================================"
}

# 检查Python环境
check_python() {
    print_info "检查Python环境..."
    
    if command -v python3 &> /dev/null; then
        PYTHON_VERSION=$(python3 --version 2>&1)
        print_success "Python环境正常: $PYTHON_VERSION"
    else
        print_error "Python3未安装或不在PATH中"
        exit 1
    fi
}

# 检查Conda环境
check_conda() {
    print_info "检查Conda环境..."
    
    if command -v conda &> /dev/null; then
        print_success "Conda环境正常"
        
        # 检查env_isaacsim环境是否存在
        if conda env list | grep -q "env_isaacsim"; then
            print_success "env_isaacsim环境存在"
        else
            print_warning "env_isaacsim环境不存在，将使用默认Python环境"
        fi
    else
        print_warning "Conda未安装，将使用系统Python环境"
    fi
}

# 检查ACADOS环境
check_acados() {
    print_info "检查ACADOS环境..."
    
    if [ -d "/home/mpc/acados" ]; then
        print_success "ACADOS目录存在: /home/mpc/acados"
    else
        print_error "ACADOS目录不存在: /home/mpc/acados"
        print_error "请先安装ACADOS"
        exit 1
    fi
}

# 检查Python依赖
check_dependencies() {
    print_info "检查Python依赖..."
    
    # 检查关键依赖
    python3 -c "
import sys
try:
    import numpy
    import matplotlib
    import pandas
    import scipy
    import casadi
    print('✓ 基础依赖检查通过')
except ImportError as e:
    print(f'✗ 依赖检查失败: {e}')
    sys.exit(1)
" 2>/dev/null
    
    if [ $? -eq 0 ]; then
        print_success "Python依赖检查通过"
    else
        print_error "Python依赖检查失败"
        print_info "正在安装依赖..."
        pip install numpy matplotlib pandas scipy casadi
    fi
}

# 设置环境变量
setup_environment() {
    print_info "设置环境变量..."
    
    # 设置ACADOS环境变量
    export ACADOS_SOURCE_DIR=/home/mpc/acados
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/mpc/acados/lib
    export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/home/mpc/acados/lib/pkgconfig
    
    # 设置ACADOS优化参数
    export ACADOS_VERBOSE='0'
    export ACADOS_WITH_QPOASES='1'
    
    print_success "环境变量设置完成"
}

# 激活Conda环境
activate_conda() {
    if command -v conda &> /dev/null && conda env list | grep -q "env_isaacsim"; then
        print_info "激活Conda环境: env_isaacsim"
        source $(conda info --base)/etc/profile.d/conda.sh
        conda activate env_isaacsim
        print_success "Conda环境激活成功"
    else
        print_info "使用系统Python环境"
    fi
}

# 运行MPC评估
run_evaluation() {
    print_info "启动MPC性能评估..."
    echo ""
    
    # 运行评估脚本
    python3 realistic_mpc_evaluation.py
    
    if [ $? -eq 0 ]; then
        print_success "MPC性能评估完成"
    else
        print_error "MPC性能评估失败"
        exit 1
    fi
}

# 显示结果目录
show_results() {
    print_info "评估结果已保存到:"
    echo "  📁 结果目录: /home/mpc/chaorui/code/Auto-Multilift_simulation/evaluation_results/"
    echo "  📊 性能报告: mpc_performance_report.csv"
    echo "  📈 频率分析: mpc_frequency_analysis.txt"
    echo "  📝 运行日志: run_info.txt"
    echo ""
    print_success "可以查看详细结果进行分析"
}

# 主函数
main() {
    print_title
    
    # 检查环境
    check_python
    check_conda
    check_acados
    check_dependencies
    
    # 设置环境
    setup_environment
    activate_conda
    
    # 运行评估
    run_evaluation
    
    # 显示结果
    show_results
    
    print_success "🎉 MPC性能评估系统运行完成！"
}

# 错误处理
trap 'print_error "脚本执行过程中发生错误，退出码: $?"' ERR

# 运行主函数
main "$@"
