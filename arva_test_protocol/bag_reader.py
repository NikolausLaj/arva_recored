"""ROS 2 bag reading helpers for offline test analysis."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

from .yaml_utils import ProtocolError


@dataclass(frozen=True)
class BagMessage:
    """A deserialized ROS 2 message with its rosbag record timestamp."""

    topic: str
    timestamp_ns: int
    message: Any


def read_messages(
    bag_directory: Path,
    topics: Iterable[str],
    storage_id: str = '',
) -> list[BagMessage]:
    """Read selected topics from a rosbag2 directory.

    ROS imports are intentionally deferred so configuration preparation can run
    independently from a ROS runtime.
    An empty storage ID lets rosbag2 detect the format from the bag metadata.
    """
    if not bag_directory.is_dir():
        raise ProtocolError(f'bag directory does not exist: {bag_directory}')

    try:
        from rclpy.serialization import deserialize_message
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise ProtocolError(
            'ROS bag analysis requires rosbag2_py, rclpy, and rosidl_runtime_py.'
        ) from error

    selected_topics = set(topics)
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_directory), storage_id=storage_id),
        ConverterOptions(
            input_serialization_format='', output_serialization_format=''
        ),
    )
    message_types = {
        topic_type.name: topic_type.type for topic_type in reader.get_all_topics_and_types()
    }
    missing_topics = selected_topics.difference(message_types)
    if missing_topics:
        missing_text = ', '.join(sorted(missing_topics))
        raise ProtocolError(f'bag does not contain required topics: {missing_text}')

    messages = []
    while reader.has_next():
        topic, serialized_data, timestamp_ns = reader.read_next()
        if topic not in selected_topics:
            continue
        message = deserialize_message(serialized_data, get_message(message_types[topic]))
        messages.append(BagMessage(topic, timestamp_ns, message))
    return messages
