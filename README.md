# iOSmsg_client

[![Build Status](https://travis-ci.org/tomas789/iOSmsg_client.svg?branch=master)](https://travis-ci.org/tomas789/iOSmsg_client)

This allows you to publish data from iOSmsg app to ROS.

## What is iOSmsg

It is a toolkit for publishing sensor data from iOS into ROS topics. It is set of two apps - [iOSmsg](https://github.com/tomas789/iOSmsg) and iOSmsg_client (this app).

### ... and how it works

The iOS app publishes sensor data to RabbitMQ topic in JSON format. Then client app reads them and publishes them directly to ROS topic.

## How to install iOSmsg_client

First, you have to install RabbitMQ and other dependencies
```
sudo apt install rabbitmq-server python-pika
```

Then install iOSmsg_app
```
cd <my-catkin-ws>/src
git clone git@github.com:tomas789/iOSmsg_client.git
cd <my-catkin-ws>
catkin_make
```
and you are done.

## How to run iOSmsg_client

```
rosrun iosmsg_client client.py
```

You can specify how to connect to RabbitMQ or where to publish data. Get more info by running
```
rosrun iosmsg_client client.py --help
```

## FAQ

**IMU message doesn't contain both accelerometer and gyroscope measuremets.**

As measurements doesn't arrive at the same time from sensor, we publish data separately. It gives you the most accurate data that are available. If you want data to arrive at the same time, you have to implement some kind of interpolation.

## Use cases

I'm using this app to debug my implementation of MSCKF, called [Tonav](https://github.com/tomas789/tonav). But there is a lot of other use cases. For example you can use it for
 * localization of your robot using your iDevice (iPhone, iPad, ...)
 * learn more about RabbitMQ
 * computer vision (_not implemented yet_)
 * and many other applications
