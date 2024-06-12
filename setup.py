from setuptools import setup, find_packages

setup(
    name='basecar',
    version='0.1.5',
    packages=find_packages(),
    install_requires=[
        # 在这里列出你的依赖包，例如：
        'numpy',
        'opencv-python',
        'rospy',
        'psutil',
        'pyserial'
    ],
    author='Linsn',
    author_email='a2675663368@163.com',
    description='A package for Ros_car project',
    long_description_content_type='text/markdown',
    python_requires='>=3.6',
)
