from distutils.core import setup

setup(name="bliss", version="0.1",
      description="Bliss",
      author="M.Guijarro (ESRF)",
      package_dir={"bliss": "bliss"},
      packages=["khoros", 'khoros.core', 'khoros.js'],
      scripts = ['bin/bliss_webserver'],) 
