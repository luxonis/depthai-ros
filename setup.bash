#!/bin/bash

HOST="https://api.woeden.com"

echo """
██     ██  ██████  ███████ ██████  ███████ ███    ██ 
██     ██ ██    ██ ██      ██   ██ ██      ████   ██ 
██  █  ██ ██    ██ █████   ██   ██ █████   ██ ██  ██ 
██ ███ ██ ██    ██ ██      ██   ██ ██      ██  ██ ██ 
 ███ ███   ██████  ███████ ██████  ███████ ██   ████ 
                                                                                                     
"""

echo "You are setting up a new robot. Please sign in."

echo -n "Email: "
read username
echo -n "Password: "
read -s password

echo

response=$(curl -s --location --request POST "$HOST/auth/login/" \
    --header 'Content-Type: application/json' \
    --data-raw "{
        \"username\": \"$username\",
        \"password\": \"$password\"
    }")
access=$(echo $response | grep -o '"access":"[^"]*' | grep -o '[^"]*$')
if [ $? -eq 0 ]; then
    echo
    echo "Awesome! You are now logged in."
else
    echo
    echo "Failed to log in. Please try again."
    exit 1
fi

echo -n "Please enter a name for the robot: "
read robot

echo -n "Please enter the version of ROS on the robot (1 or 2): "
read ros_version

response=$(curl -s --location --request POST "$HOST/robot/" \
    --header "Authorization: Bearer $access" \
    --header 'Content-Type: application/json' \
    --data-raw "{
        \"name\": \"$robot\",
        \"ros_version\": \"ROS$ros_version\"
    }")
robot_id=$(echo $response | grep -o '"id":[0-9]*' | grep -m 1 -o '[0-9]*$')

if [ $? -eq 0 ]; then
    mkdir -p $HOME/woeden
    echo $response > $HOME/woeden/config
    echo
    echo "Your new robot, $robot, is now registered in Woeden."
else
    echo
    echo "Failed to initialize your new robot. Please try again."
    exit 1
fi

echo "Please complete registration of the robot here: http://app.woeden.com/app/robot/$robot_id/settings/"

echo
echo "To use Woeden, please be sure to run our Docker container image. You can accomplish this by adding"
echo "the following service to the docker-compose file of your robotics project."

image_name=$([ "$ros_version" = 1 ] && echo "public.ecr.aws/woeden/ros1-agent" || echo "public.ecr.aws/woeden/agent")

echo """
> woeden_agent:
>     image: $image_name
>     network_mode: host
>     ipc: host
>     restart: always
>     volumes:
>       - ~/woeden:/woeden
"""

echo "Alternatively, you can simply run the command below. Please note that for the agent to detect topics"
echo "running directly on your host (not containerized), you will need to run those ROS $ros_version programs as root."

echo
echo "> docker run -d --net=host --ipc=host -v ~/woeden:/woeden --restart always $image_name:latest"
