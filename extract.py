import os
import click
import pathlib
from typing import Optional
from collections import deque
from cyber_record.record import Record
from record_msg.parser import PointCloudParser
from google.protobuf.json_format import MessageToDict
import datetime

@click.group()
def main():
    """main"""


def extract_pose_key_fields_from_msg(msg, target_field='localization'):
    """
    """
    pose_data = MessageToDict(msg).get(target_field, {})
    orientation = pose_data.get('orientation', {})
    qw = orientation.get('qw', 1.0)
    qx = orientation.get('qx', 0.0)
    qy = orientation.get('qy', 0.0)
    qz = orientation.get('qz', 0.0)

    rot_matrix = [
        [
            qw**2 + qx**2 - qy**2 - qz**2, 2 * (qx * qy - qw * qz),
            2 * (qx * qz + qw * qy)
        ],
        [
            2 * (qx * qy + qw * qz), qw**2 - qx**2 + qy**2 - qz**2,
            2 * (qy * qz - qw * qx)
        ],
        [
            2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx),
            qw**2 - qx**2 - qy**2 + qz**2
        ],
    ]

    position = pose_data.get('position', {})
    x = position.get('x', 0.0)
    y = position.get('y', 0.0)
    z = position.get('z', 0.0)
    return (rot_matrix, x, y, z)


@main.command()
@click.option('--input_path',
              type=str,
              required=True,
              help='directory of the record files')
@click.option('--output_dir',
              type=str,
              required=True,
              help='directory to save the extracted pose and pcd files')
@click.option('--pose_channel',
              type=str,
              default='/apollo/sensor/gnss/odometry',
              help='target channel to extract pose data')
@click.option('--pose_field',
              type=str,
              default='localization',
              help='target field in the message to extract pose data')
@click.option('--pointcloud_channel',
              type=str,
              default='/apollo/sensor/vanjeelidar/up/PointCloud2',
              help='target channel to extract point cloud data')
@click.option(
    '--pointcloud_mode',
    type=str,
    default='ascii',
    help='mode for saving point cloud data, e.g., "ascii" or "binary"')
@click.option('--initial_x',
              type=float,
              required=False,
              help='initial x offset')
@click.option('--initial_y',
              type=float,
              required=False,
              help='initial y offset')
@click.option('--initial_z',
              type=float,
              required=False,
              help='initial z offset')
def extract(
    input_path: str,
    output_dir: str,
    pose_channel: str = '/apollo/sensor/gnss/odometry',
    pose_field: str = 'localization',
    pointcloud_channel: str = '/apollo/sensor/vanjeelidar/up/PointCloud2',
    pointcloud_mode: str = 'ascii',
    initial_x: Optional[float] = None,
    initial_y: Optional[float] = None,
    initial_z: Optional[float] = None,
):
    """Extract both pose and point cloud data from record files."""
    pcds_dir = os.path.join(output_dir, 'pcds')
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    if not os.path.exists(pcds_dir):
        os.makedirs(pcds_dir)

    pose_msg_queue = deque(maxlen=1000)
    pointcloud_msg_queue = deque()

    initial_pose_setted = False
    if initial_x and initial_y and initial_z:
        initial_pose_setted = True

    def _bisect(queue, t):
        """Find the index to insert a timestamp in a sorted queue."""
        low, high = 0, len(queue)
        while low < high:
            mid = (low + high) // 2
            if queue[mid][0] < t:
                if low == mid:
                    return low
                low = mid
            elif queue[mid][0] > t:
                high = mid
            else:
                return mid
        return low

    # TODO(All): merge records or detect the real order of records
    files = sorted([
        f
        for f in [os.path.join(input_path, x) for x in os.listdir(input_path)]
        if os.path.isfile(f)
    ])
    pose_out_file = open(pathlib.Path(output_dir) / 'pose.txt', 'w')
    pointcloud_parser = PointCloudParser(
        pathlib.Path(output_dir) / 'pcds', True, '.pcd')
    for file in files:
        with Record(file) as record:
            for channel, msg, t in record.read_messages(
                    topics=[pose_channel, pointcloud_channel]):
                # print(f'processing {file}, channel: {channel}, t: {t}')
                if channel == pose_channel:
                    pose_msg_queue.append((t, msg))
                    print(
                        f'append pose message to queue, length: {len(pose_msg_queue)}, from {pose_msg_queue[0][0]} to {pose_msg_queue[-1][0]}'
                    )
                    pc_comsumed_idx = -1
                    for idx in range(len(pointcloud_msg_queue)):
                        pc_time, _ = pointcloud_msg_queue[idx]
                        closest_pose_idx = _bisect(pose_msg_queue, pc_time)
                        if closest_pose_idx == len(pose_msg_queue) - 1:
                            # If the closest pose is the last one, the real
                            # closest pose may in the future, so we skip it.
                            break
                        pose_time, pose_msg = pose_msg_queue[closest_pose_idx]
                        print(
                            f'closest pose idx {closest_pose_idx}, time: {pose_time}, diff {pc_time - pose_time} ns'
                        )
                        # TODO(All): check time difference
                        time_str = datetime.datetime.fromtimestamp(
                            pc_time /
                            1e9).strftime('%Y-%m-%d-%H-%M-%S-%f')[:-3]
                        rot_matrix, x, y, z = extract_pose_key_fields_from_msg(
                            pose_msg, target_field=pose_field)
                        print(rot_matrix, x, y, z)
                        # save pose
                        if not initial_pose_setted:
                            # 设置初始平移向量
                            initial_x = x
                            initial_y = y
                            initial_z = z
                            initial_pose_setted = True

                        # 对平移向量进行偏移处理
                        x_offset = x - initial_x
                        y_offset = y - initial_y
                        z_offset = z - initial_z

                        # 格式化写入文件（匹配OpenCalib格式）
                        line = ''.join([
                            f'{time_str} ',
                            ' '.join([f'{i: .9f}' for i in rot_matrix[0]]),
                            f' {x_offset: .9f} ',
                            ' '.join([f'{i: .9f}' for i in rot_matrix[1]]),
                            f' {y_offset: .9f} ',
                            ' '.join([f'{i: .9f}' for i in rot_matrix[2]]),
                            f' {z_offset: .9f}\n',
                        ])
                        pose_out_file.write(line)
                        pc_comsumed_idx = idx

                    print(f'pc_comsumed_idx: {pc_comsumed_idx}')

                    if pc_comsumed_idx >= 0:
                        # Remove consumed point cloud messages
                        pointcloud_msg_queue = deque(
                            list(pointcloud_msg_queue)[pc_comsumed_idx + 1:])

                elif channel == pointcloud_channel:
                    time_str = datetime.datetime.fromtimestamp(
                        t / 1e9).strftime('%Y-%m-%d-%H-%M-%S-%f')[:-3]
                    pointcloud_parser.parse(msg,
                                            file_name=f'{time_str}',
                                            mode=pointcloud_mode)
                    pointcloud_msg_queue.append((t, msg))
                    print(
                        f'append pointcloud message to queue, length: {len(pointcloud_msg_queue)}, from {pointcloud_msg_queue[0][0]} to {pointcloud_msg_queue[-1][0]}'
                    )


if __name__ == '__main__':
    main()
