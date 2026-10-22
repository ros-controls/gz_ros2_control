# Copyright 2026 ros2_control Maintainers
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import psutil


def stop_gazebo(timeout=10):
    """Stop the Gazebo processes gracefully, kill the ones still alive after timeout seconds."""
    gz_procs = [
        proc for proc in psutil.process_iter()
        # ruby and 'gz sim' up to version 9 of gz-sim, gz-sim from version 10
        if proc.name() == 'ruby' or 'gz sim' in proc.name() or 'gz-sim' in proc.name()
    ]
    # Stop Gazebo gracefully so it can write its coverage data
    for proc in gz_procs:
        proc.terminate()
    _, alive = psutil.wait_procs(gz_procs, timeout=timeout)
    for proc in alive:
        proc.kill()
