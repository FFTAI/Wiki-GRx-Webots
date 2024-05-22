from setuptools import setup, find_packages

setup(
    name='robot_rcs_gr',
    version='0.0.1',
    description='Robotics Control System for the General Robotics',
    author='Fourier Intelligence',

    packages=find_packages(),
    install_requires=[
        'numpy',
        'torch',
        # for IMU
        # https://pyserial.readthedocs.io/en/latest/pyserial_api.html
        'pyserial==3.5',  # python3.8 无法使用 pyserial==2.7
        # for joystick
        'pygame',
        'matplotlib',
        'nuitka',
        # gRPC
        # 'grpcio',
        # 'grpcio-tools',
        'pyyaml',
    ],
    python_requires='==3.11.*',  # 这就是设置 Python 版本的地方
)
