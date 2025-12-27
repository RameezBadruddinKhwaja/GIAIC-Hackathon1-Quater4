---
sidebar_position: 2
title: "Week 3-5: ROS 2 Fundamentals"
---

# ROS 2 Fundamentals

## Learning Objectives
- Understand the Robot Operating System 2 (ROS 2) architecture
- Learn about nodes, topics, services, and actions
- Explore ROS 2 packages and workspaces
- Practice basic ROS 2 commands and tools

## What is ROS 2?

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms. Unlike traditional frameworks, ROS 2 provides a middleware layer that enables communication between different parts of a robot system.

## Architecture Overview

### DDS Middleware
ROS 2 uses DDS (Data Distribution Service) as the underlying communication mechanism. DDS provides reliable, real-time communication between different parts of a robot system. This middleware approach allows ROS 2 nodes to communicate across different programming languages and operating systems.

### Nodes
A ROS 2 node is a process that performs computation. Nodes are organized into packages for better management and distribution. Each node can publish to or subscribe to multiple topics, call services, or provide services to other nodes.

### Topics and Messages
Nodes communicate with each other using topics. A topic is a named bus over which nodes exchange messages. Messages are data structures that are passed between nodes. Topics implement a publish-subscribe communication pattern where publishers send messages to a topic and subscribers receive messages from a topic.

### Services and Actions
Services provide a request-response communication pattern where a client sends a request and waits for a response. Actions are similar to services but are designed for long-running tasks that may need to be canceled or provide feedback during execution.

## Practical Examples

### Creating a Node
To create a ROS 2 node, you typically extend the rclcpp::Node or rclpy.Node class and implement the necessary functionality. Nodes can be written in multiple programming languages including C++, Python, and others.

### Package Structure
ROS 2 packages follow a specific structure with package.xml and CMakeLists.txt (for C++) or setup.py (for Python) files. Packages contain source code, configuration files, launch files, and other resources needed for the node to function.

## Summary

ROS 2 provides a robust framework for developing complex robot systems. Its modular architecture allows for flexible communication patterns and easy integration of different components. Understanding ROS 2 fundamentals is essential for working with Physical AI systems.

## Assessment

1. Explain the difference between topics, services, and actions in ROS 2.
2. Describe the role of DDS in ROS 2 architecture.
3. Create a simple ROS 2 node that publishes a message to a topic.