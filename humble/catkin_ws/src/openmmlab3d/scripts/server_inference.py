#!/usr/bin/env python3

from openmmlab3d.srv import Inference

import rclpy
from rclpy.node import Node

class InferenceSrv(Node):

	def __init__(self):
		super().__init__('inferenceSrv')
		self.srv = self.create_service(Inference, 'inference', self.makeAnInference_callback)

	def makeAnInference_callback(self, request, response):
		try:
			response.result = int(request.file.data) + 5
		except ValueError:
			response.result = -1

		return response

def main():
	rclpy.init()

	print('Setting up node...')

	inferenceSrv = InferenceSrv()

	print('Inferecer node is ready to perform')

	rclpy.spin(inferenceSrv)

	rclpy.shutdown()

if __name__ == '__main__':
	main()
