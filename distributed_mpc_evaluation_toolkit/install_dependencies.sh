#!/bin/bash

# =============================================================================
# 依赖安装脚本
# Dependencies Installation Script
# =============================================================================

set -euo pipefail

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查Python环境
check_python() {
    log_info "检查Python环境..."
    
    if command -v python3 &> /dev/null; then
        PYTHON_VERSION=$(python3 --version 2>&1 | cut -d' ' -f2)
        log_success "找到Python3: $PYTHON_VERSION"
    else
        log_error "未找到Python3，请先安装Python3"
        exit 1
    fi
    
    if command -v pip3 &> /dev/null; then
        log_success "找到pip3"
    else
        log_warning "未找到pip3，尝试使用python3 -m pip"
        PIP_CMD="python3 -m pip"
    fi
}

# 安装Python包
install_python_packages() {
    log_info "安装Python依赖包..."
    
    # 基础包
    local packages=(
        "numpy"
        "matplotlib"
        "pandas"
        "scipy"
        "scikit-learn"
    )
    
    for package in "${packages[@]}"; do
        log_info "安装 $package..."
        if pip3 install "$package" --user; then
            log_success "$package 安装成功"
        else
            log_error "$package 安装失败"
            exit 1
        fi
    done
}

# 验证安装
verify_installation() {
    log_info "验证安装..."
    
    python3 -c "
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import scipy
import sklearn
print('所有Python包导入成功')
print(f'NumPy版本: {np.__version__}')
print(f'Matplotlib版本: {plt.matplotlib.__version__}')
print(f'Pandas版本: {pd.__version__}')
print(f'SciPy版本: {scipy.__version__}')
print(f'Scikit-learn版本: {sklearn.__version__}')
" || {
        log_error "Python包验证失败"
        exit 1
    }
    
    log_success "Python包验证通过"
}

# 创建虚拟环境（可选）
create_venv() {
    if [ "${1:-}" = "--venv" ]; then
        log_info "创建Python虚拟环境..."
        
        if python3 -m venv mpc_eval_env; then
            log_success "虚拟环境创建成功"
            log_info "激活虚拟环境: source mpc_eval_env/bin/activate"
            log_info "然后重新运行安装脚本"
        else
            log_error "虚拟环境创建失败"
            exit 1
        fi
    fi
}

# 主函数
main() {
    echo "=========================================="
    echo "分布式MPC性能评估工具依赖安装"
    echo "Dependencies Installation for MPC Performance Evaluation"
    echo "=========================================="
    echo
    
    check_python
    install_python_packages
    verify_installation
    
    echo
    echo "=========================================="
    log_success "依赖安装完成！"
    echo "=========================================="
    echo
    echo "现在可以运行性能评估脚本："
    echo "  ./quick_test_mpc_performance.sh"
    echo "  ./evaluate_distributed_mpc_performance.sh"
    echo
}

# 运行主函数
main "$@"
