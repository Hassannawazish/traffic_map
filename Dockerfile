# From osrf/ros:noetic-desktop

# RUN apt-get update
# RUN apt-get install -y git && apt-get install -y python3-pip

# syntax=docker/dockerfile:1
FROM python:3.7-alpine
WORKDIR /code
ENV FLASK_APP=app.py
ENV FLASK_RUN_HOST=0.0.0.0