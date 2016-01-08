from distutils.core import setup

setup(name="bliss", version="0.1",
      description="BeamLine Instrumentation Support Software",
      author="BCU (ESRF)",
      package_dir={"bliss": "bliss"},
      packages=['bliss', 'bliss.comm', 'bliss.comm.embl', 'bliss.comm.gpib', 'bliss.common', 'bliss.config', 'bliss.config.conductor', 'bliss.config.conductor.web', 'bliss.config.motors', 'bliss.config.plugins', 'bliss.config.redis', 'bliss.controllers', 'bliss.controllers.motors', 'bliss.controllers.motors.libicepap', 'bliss.controllers.motors.libicepap.deep', "bliss.shell", "bliss.shell.interpreter", "bliss.shell.web", "bliss.shell.interpreter"],
      package_data={"bliss.config.redis": ["redis.conf"],
                    "bliss.config.plugins": ["*.html"],
                    "bliss.config.conductor.web": ["*.html",
                                             "css/*.*",
                                             "css/jstree/*.*",
                                             "js/*.*",
                                             "res/*.*"],
                    'bliss.shell.web':['*.html', 'css/*.css', "js/*.js"]},
      scripts=["bin/beacon-server", "bin/bliss", 'bin/bliss_webserver'])

