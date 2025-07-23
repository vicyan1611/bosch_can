## How to Run Simulator

- Turn on vcan

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

- Turn on virtualenv and install requirement.txt
- Run webserver

```bash
python webserver/main.py
```

- Run detector

```bash
python Simulating/main.py
```

- Run Simulator

```bash
python Simulating/simulate_can_messages.py
```
