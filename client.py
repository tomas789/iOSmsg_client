#!/usr/bin/env python

import sys
import argparse
import pika
import json
import datetime
import rospy
import pytz
from sensor_msgs.msg import Imu 


class iOSmsgClient:
    """iOSmsgClient: Publish IMU data from iOS to ROS"""

    def __init__(self):
        self.publisher = None
        parser = self.arg_parser()
        self.args = parser.parse_args(rospy.myargv()[1:])

    def arg_parser(self):
        arg_fmt = argparse.RawDescriptionHelpFormatter
        parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                         description=iOSmsgClient.__doc__)
        parser.add_argument(
            '-s', '--host', type=str, default='localhost',
            help='RabbitMQ broker hostname'
        )
        parser.add_argument(
            '-p', '--port', type=int, default=5672,
            help='RabbitMQ broker port'
        )
        parser.add_argument(
            '--vhost', type=str, default='/',
            help='RabbitMQ broker vhost'
        )
        parser.add_argument(
            '-u', '--user', type=str, default='guest',
            help='RabbitMQ broker username'
        )
        parser.add_argument(
            '--password', type=str, default='guest',
            help='RabbitMQ broker password'
        )
        parser.add_argument(
            '-e', '--exchange', type=str, default='iosmsg',
            help='RabbitMQ broker exchange'
        )
        parser.add_argument(
            '-t', '--topic', type=str, default='/iosmsg/imu_data',
            help='topic name'
        )
        return parser

    def connect(self):
        credentials = pika.PlainCredentials(self.args.user, self.args.password)
        conn_params = pika.ConnectionParameters(self.args.host, self.args.port, self.args.vhost, credentials)
        return pika.BlockingConnection(conn_params)

    def handleData(self, ch, method, properties, body):
        data = json.loads(body.decode())
        msg = Imu()
        msg.header.stamp = rospy.Time.from_sec(data['time'])
        if data['sensor'] == 'accelerometer':
            msg.linear_acceleration.x = data['x']
            msg.linear_acceleration.y = data['y']
            msg.linear_acceleration.z = data['z']
        elif data['sensor'] == 'gyroscope':
            msg.angular_velocity.x = data['x']
            msg.angular_velocity.y = data['y']
            msg.angular_velocity.z = data['z']
        self.publisher.publish(msg)

    def run(self):
        rospy.init_node('iosmsg_client', anonymous=True)

        connection = self.connect()
        channel = connection.channel()
        channel.exchange_declare(exchange=self.args.exchange, type='fanout')
        result = channel.queue_declare(exclusive=True)
        queue_name = result.method.queue
        channel.queue_bind(exchange=self.args.exchange, queue=queue_name)
        channel.basic_consume(self.handleData, queue=queue_name, no_ack=True)

        self.publisher = rospy.Publisher(self.args.topic, Imu, queue_size=10)

        rospy.loginfo('Waiting for data. To exit press CTRL+C')
        channel.start_consuming()


if __name__ == '__main__':
    client = iOSmsgClient()
    client.run()
