"""Bus communication

This module provides a framework different modules to send data to
each other while also decoupling the data. The api was inspired by rospy.
The advantage of this is that systems don't have to directly interact with
each other. This makes them easier to understand and debug. 

The design is a multiple producer multiple consumer event system.
Modules create a producer for events, and subscribers to consume events.
Any module can produce/consume events for any topic name. This is the
basis for inter-system communication using this module.

This module provides 2 main classes:
- Publisher: used to send data into the bus.
- Subscriber: used to consume data from the bus.
    - subscribe: helper function for creating subscribers.

It also provides some debugging utilities:
- describe_topic
- describe_node
- get_topics
- publish_once
"""


import sys
import os
from typing import TypeVar, Callable, Generic
T = TypeVar('T')


def currentframe(): return sys._getframe(1)


def is_internal_frame(frame):
    """Checks if a frame is part of bus.py"""
    m_name = frame.f_globals['__name__']
    return m_name == __name__


def generate_name():
    """This generates a module name using stack frames."""
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
    """
    Acts like a mail man which takes data from publishers and sends it
    to subscribers. It also provides some debugging tools like keeping
    track of what publishers/subscribers exist and basic type checking.
    """

    def __init__(self, topic_name: str, topic_type: type):
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.observers = dict()
        self.publishers = []
        self.queue = []

    def __repr__(self):
        return f'Topic({self.topic_name}, {self.topic_type})'

    def register_observer(self, callback: Callable[[T], None], name: str):
        """Register a `callback` function used to send data to a subscriber"""
        if name in self.observers:
            raise IndexError(f'{name} is already registered to {self}')

        self.observers[name] = callback

        for data in self.queue:
            callback(data)

    def register_publisher(self, name: str):
        """Register the name of a publisher"""
        if name in self.publishers:
            raise IndexError(f'{name} is already a publisher for {self}')
        self.publishers.append(name)

    def broadcast(self, data: T):
        """Sends data to subscribers."""
        if not isinstance(data, self.topic_type):
            raise TypeError(
                f'{self} passed an invalid type {data}({type(data)})')

        self.queue.append(data)
        while len(self.queue) > 10:
            self.queue.pop(0)

        for name in self.observers:
            self.observers[name](data)


class Publisher(Generic[T]):
    """
    This class is used to send data into the bus.

    If a name is not provided, it defaults to the name of the module.

    Example:
    ```
    publisher = bus.Publisher('topic_name', str)
    publisher.publish('hello there')
    ```
    """

    def __init__(self, topic_name: str, topic_type: type, name: str | None = None):
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
        """Send `data` to the topic to be broadcast to subscribers."""
        self.topic.broadcast(data)


topics = {}


class Subscriber(Generic[T]):
    """
    This class is used to add a callback function to the bus for some topic.

    If a name is not provided, it defaults to the name of the module.

    Example:
    ```
    def say_hi(name):
        print(name)

    bus.Subscriber('topic_name', str, say_hi)
    ```
    """

    def __init__(self, topic_name, topic_type: type, callback: Callable[[T], None], name: str | None = None):
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
    """Prints publishers and subscribers associated with a topic."""
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
    """Prints publishers and subscribers that share a node.

    Generally this will print all publishers and subscribers located
    in the same module file.
    """
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
    """Returns a list of used topic names."""
    return [topic for topic in topics]


def inspect(topic_name: str, topic_type: type):
    """Creates a subscriber that simply prints anything sent to a topic."""
    if not hasattr(inspect, 'count'):
        inspect.count = 0

    inspect.count += 1

    def log(data):
        print(data)

    Subscriber(topic_name, topic_type, log, f'inspector@{inspect.count}')


def publish_once(topic_name: str, topic_type: type, data: T):
    """Creates a one off publisher that sends data once.

    This could be used to initialize data.
    """
    pub = Publisher(topic_name, topic_type)
    pub.publish(data)


def subscribe(topic_name: str, topic_type: type, name: str | None = None):
    """
    This decorator function can be used to register a function as
    a subscriber.

    This exists to make registering a subscriber less ugly.

    If a name is not provided, it defaults to the name of the module.

    Example:
    ```
    @bus.subscribe('topic_name', str)
    def say_hi(name):
        print(name)
    ```
    """
    def wrapper(callback: Callable[[T], None]):
        Subscriber(topic_name, topic_type, callback, name=name)
        return callback
    return wrapper
