# EgoLanes - Open AD Kit Demo

Containerized EgoLanes Demo, semantic segmentation of driving lanes.

## Prerequisites

- Download the [EgoLanes PyTorch model weights](https://github.com/autowarefoundation/autoware.privately-owned-vehicles/tree/main/Models#egolanes---semantic-segmentation-of-driving-lanes) and place it in the `model-weights` directory with the name `egolanes.pth`.

    ```bash
    mkdir -p model-weights
    curl "https://drive.usercontent.google.com/download?id=1Njo9EEc2tdU1ffo8AUQ9mjwuQ9CzSRPX&confirm=xxx" -o model-weights/egolanes.pth
    ```

## Usage

```bash
./launch-egolanes.sh
```

## Output

After the container is running, you can access the visualization by opening the following URL in your browser:

<http://127.0.0.1:6080/vnc.html?resize=scale&autoconnect=true&password=visualizer>

> **Note:** Use `127.0.0.1` instead of `localhost`. On systems where `localhost` resolves to IPv6 (`::1`), the connection will fail as Podman's `pasta` network backend only handles IPv4.

The output shows semantic segmentation of the driving lanes of the input video in real-time.
