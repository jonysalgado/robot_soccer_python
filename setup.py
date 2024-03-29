from setuptools import setup

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
  name = 'robot_soccer_python',         # How you named your package folder (MyLib)
  packages = ['robot_soccer_python'],   # Chose the same as "name"
  version = '1.0.7',      # Start with a small number and increase it with every change you make
  license='MIT',        # Chose a license from here: https://help.github.com/articles/licensing-a-repository
  description = 'A robot soccer simulation 2D environment for python.',   # Give a short description about your library
  long_description=long_description,
  long_description_content_type="text/markdown",
  author = 'Jony Salgado',                   # Type in your name
  author_email = 'jonysalgadofilho@gmail.com',      # Type in your E-Mail
  url = 'https://github.com/jonysalgado/robot_soccer_python',   # Provide either the link to your github or to your website
  download_url = 'https://github.com/jonysalgado/robot_soccer_python/archive/refs/tags/v_01.7.tar.gz',    # I explain this later on
  keywords = ['SIMULATION', 'SOCCER', 'ROBOT', 'AI', 'ENVIRONMENT', 'PYTHON'],   # Keywords that define your package best
  install_requires=[            # I get to this in a second
        'numpy',
        'pygame'
     ],
  classifiers=[
    'Development Status :: 3 - Alpha',      # Chose either "3 - Alpha", "4 - Beta" or "5 - Production/Stable" as the current state of your package
    'Intended Audience :: Developers',      # Define that your audience are developers
    'Topic :: Software Development :: Build Tools',
    'License :: OSI Approved :: MIT License',   # Again, pick a license
    'Programming Language :: Python :: 3',      #Specify which pyhton versions that you want to support
    'Programming Language :: Python :: 3.4',
    'Programming Language :: Python :: 3.5',
    'Programming Language :: Python :: 3.6',
    'Programming Language :: Python :: 3.7',
    'Programming Language :: Python :: 3.8'
  ],
)