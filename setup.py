from setuptools import setup
# from distutils.core import setup

setup(name='planarobot',
      packages=['planarobot'],
      version='0.0.3',
      description='Basic planar robotic arm with arbitrary number of links',
      author='Theodore Tsitsimis',
      author_email='th.tsitsimis@gmail.com',
      url='https://github.com/tsitsimis/planar-robot',
      download_url='https://github.com/tsitsimis/planarobot/archive/0.0.3.tar.gz',
      keywords=['robotics', 'simulation', 'path-planning'],
      license='MIT',
      classifiers=[
          'Development Status :: 3 - Alpha',

          'Intended Audience :: Developers',
          'Topic :: Software Development :: Build Tools',

          'License :: OSI Approved :: MIT License',

          'Programming Language :: Python :: 3'
      ],
      install_requires=[
          'numpy',
      ],
      zip_safe=False
      )
