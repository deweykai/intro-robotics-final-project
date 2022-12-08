import sys
import os
from typing import TypeVar, Callable, Generic
T = TypeVar('T')


def currentframe(): return sys._getframe(1)


def is_internal_frame(frame):
    m_name = frame.f_globals['__name__']
    return m_name == __name__


def generate_name():
    f = currentframe()
    stacklevel = 1
    while stacklevel > 0:
        f_next = f.f_back
        if f_next is None:
            break
        f = f_next
        if not is_internal_frame(f):
            stacklevel -= 1

    fn = f.f_globals['__name__']
    return fn


class Topic(Generic[T]):
    def __init__(self, topic_name: str, topic_type: type):
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.observers = dict()
        self.publishers = []
        self.queue = []

    def __repr__(self):
        return f'Topic({self.topic_name}, {self.topic_type})'

    def register_observer(self, callback: Callable[[T], None], name: str):
        if name in self.observers:
            raise IndexError(f'{name} is already registered to {self}')

        self.observers[name] = callback

        for data in self.queue:
            callback(data)

    def register_publisher(self, name: str):
        if name in self.publishers:
            raise IndexError(f'{name} is already a publisher for {self}')
        self.publishers.append(name)

    def broadcast(self, data: T):
        if not isinstance(data, self.topic_type):
            raise TypeError(
                f'{self} passed an invalid type {data}({type(data)})')

        self.queue.append(data)
        while len(self.queue) > 10:
            self.queue.pop(0)

        for name in self.observers:
            self.observers[name](data)


class Publisher(Generic[T]):
    def __init__(self, topic_name: str, topic_type: type, name=None):
        if name is None:
            name = generate_name()
        self.name = name

        global topics
        if topic_name not in topics:
            topics[topic_name] = Topic(topic_name, topic_type)

        topic = topics[topic_name]

        if topic.topic_type != topic_type:
            raise TypeError(
                f'Tried to create publisher ({topic_type}) for ({topic})')

        topic.register_publisher(name)
        self.topic = topic

    def publish(self, data: T):
        self.topic.broadcast(data)


topics = {}


class Subscriber(Generic[T]):
    def __init__(self, topic_name, topic_type: type, callback: Callable[[T], None], name=None):
        if name is None:
            name = generate_name()

        global topics
        if topic_name not in topics:
            topics[topic_name] = Topic(topic_name, topic_type)

        topic = topics[topic_name]
        if topic.topic_type != topic_type:
            raise TypeError(
                f'Tried to create subscriber ({topic_type}) for ({topic})')
        topic.register_observer(callback, name)


def describe_topic(topic_name: str):
    if topic_name not in topics:
        print(f'{topic_name} is not a used topic')
        return

    topic = topics[topic_name]
    print('=' * 3, f'TOPIC: {topic_name}', '=' * 3)

    print('PUBLISHERS:')
    for name in topic.publishers:
        print(f'- {name}')

    print('\n')
    print('SUBSCRIBERS:')
    for name in topic.observers:
        print(f'- {name}')


def describe_node(node_name: str):
    print('=' * 3, f'NODE: {node_name}', '=' * 3)

    print('PUBLISHERS:')
    for topic_name in topics:
        topic = topics[topic_name]
        if node_name in topic.publishers:
            print(f'- {topic_name}')

    print('\n')
    print('SUBSCRIBERS:')
    for topic_name in topics:
        topic = topics[topic_name]
        if node_name in topic.observers:
            print(f'- {topic_name}')


def get_topics():
    return [topic for topic in topics]


def inspect(topic_name: str, topic_type: type):
    if not hasattr(inspect, 'count'):
        inspect.count = 0

    inspect.count += 1

    def log(data):
        print(data)

    Subscriber(topic_name, topic_type, log, f'inspector@{inspect.count}')


def publish_once(topic_name: str, topic_type: type, data: T):
    pub = Publisher(topic_name, topic_type)
    pub.publish(data)


def subscribe(topic_name: str, topic_type: type, name=None):
    def wrapper(callback: Callable[[T], None]):
        Subscriber(topic_name, topic_type, callback, name=name)
        return callback
    return wrapper
