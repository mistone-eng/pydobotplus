import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name='pydobot2',
    packages=['pydobot'],
    version='0.1.0',
    description='Python library for Dobot Magician',
    long_description=long_description,
    long_description_content_type='text/markdown',
    author='Zdenek Materna',
    author_email='imaterna@fit.vut.cz',
    url='https://github.com/zdenekm/pydobot',
    keywords=['dobot', 'magician', 'robotics', 'm1'],
    classifiers=[],
    install_requires=[
        'pyserial==3.4'
    ]
)
