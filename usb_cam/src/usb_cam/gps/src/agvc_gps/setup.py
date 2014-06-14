#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.package import parse_package_for_distutils

package_info = parse_package_for_distutils()
package_info['packages'] = ['nmea_gps_driver']
package_info['package_dir'] = {'*': 'src'}
package_info['install_requires'] = []

setup(**package_info)
