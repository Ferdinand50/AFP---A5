
# -----------------------------------------------------------------------------
# Copyright (c) 2022, Lucid Vision Labs, Inc.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
# OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
# BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
# ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
# -----------------------------------------------------------------------------

import sys

from arena_api.__future__.save import Writer
from arena_api.enums import PixelFormat
from arena_api.system import system

import open3d as o3d



def validate_device(device):

	# validate if Scan3dCoordinateSelector node exists.
	# If not, it is (probably) not a Helios Camera running the example
	try:
		scan_3d_operating_mode_node = device. \
			nodemap['Scan3dOperatingMode'].value
	except (KeyError):
		print(f'Scan3dCoordinateSelector node is not found. ' \
			f'Please make sure that Helios device is used for the example.\n')
		sys.exit()


def get_ply():

	devices = system.create_device()
	if not len(devices):
		raise Exception(f'No device found!\n'
						f'Please connect a device and run the example again.')
	print(f'Created {len(devices)} device(s)')

	device = devices[0]

	validate_device(device)

	tl_stream_nodemap = device.tl_stream_nodemap
	print(f'Device used in the example:\n\t{device}')

	# Enable stream auto negotiate packet size
	tl_stream_nodemap['StreamAutoNegotiatePacketSize'].value = True

	# Enable stream packet resend
	tl_stream_nodemap['StreamPacketResendEnable'].value = True

	device.nodemap['PixelFormat'].value = PixelFormat.Coord3D_ABC16
 
	device.nodemap['ExposureTimeSelector'].value = "Exp250Us"
	device.nodemap['ConversionGain'].value = "Low"
	device.nodemap['Scan3dConfidenceThresholdMin'].value = 100
	device.nodemap['Scan3dImageAccumulation'].value = 16

	with device.start_stream():
		print('Stream started')

		buffer = device.get_buffer()
		print(f'Image buffer received')
  
		writer = Writer()

		writer.save(buffer, 'I_AM_A_3D_BECAUSE_OF_MY_EXTENSION.ply')

		print(f'Image saved {writer.saved_images[-1]}')
  
		ply_path = writer.saved_images[-1]
		point_cloud = o3d.io.read_point_cloud(ply_path)
		# o3d.visualization.draw_geometries([point_cloud])

		device.requeue_buffer(buffer)
		print(f'Image buffer requeued')


	system.destroy_device()
	print('Destroyed all created devices')
 
	return point_cloud


if __name__ == '__main__':
	get_ply()