from setuptools import setup

setup(name='agt_yhs_adapter', version='0.1.0', packages=['agt_yhs_adapter'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/agt_yhs_adapter']),
                  ('share/agt_yhs_adapter', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='AGT', maintainer_email='contact@aldoubt.com',
      description='Twist -> YHS TK-mid CtrlCmd platform adapter', license='Apache-2.0',
      entry_points={'console_scripts': [
          'yhs_cmd_vel_bridge = agt_yhs_adapter.cmd_vel_bridge:main']})
