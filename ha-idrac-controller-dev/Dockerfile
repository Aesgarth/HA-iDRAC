# HA-iDRAC/ha-idrac-controller/Dockerfile
# Use an official Python slim image based on Debian Bookworm
FROM python:3.11-slim-bookworm

# Set the default shell for RUN commands to bash for consistency
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Install system dependencies: ipmitool, snmp, bash, and jq
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ipmitool \
    snmp \
    bash \
    jq \  
 && rm -rf /var/lib/apt/lists/* \
 && snmpwalk -V

# Set the working directory
WORKDIR /app

# Copy requirements.txt first to leverage Docker's build cache
COPY app/requirements.txt ./
# Ensure pip is up-to-date, then install requirements
RUN python3 -m pip install --upgrade pip && \
    pip install --no-cache-dir -r requirements.txt

# Copy the rest of your application code from the 'app' subdirectory
COPY app/ /app/

# Copy your run.sh script, make it executable, and place it where it's easy to call
COPY run.sh /usr/local/bin/run.sh
RUN chmod +x /usr/local/bin/run.sh

# Set the command to run when the container starts
CMD ["/usr/local/bin/run.sh"]