from typing import TypeVar, Callable
T = TypeVar('T')


class Topic:
    def __init__(self, topic_name: str, topic_type: type):
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.observers = []

    def __repr__(self):
        return f'Topic({self.topic_name}, {self.topic_type})'

    def register_observer(self, callback: Callable[[T], None]):
        self.observers.append(callback)

    def broadcast(self, data: T):
        if not isinstance(data, self.topic_type):
            raise TypeError(
                f'{self} passed an invalid type {data}({type(data)})')
        for observer in self.observers:
            observer(data)


class Publisher:
    def __init__(self, topic_name: str, topic_type: type):
        global topics
        if topic_name not in topics:
            topics[topic_name] = Topic(topic_name, topic_type)

        topic = topics[topic_name]
        if topic.topic_type != topic_type:
            raise TypeError(
                f'Tried to create publisher ({topic_type}) for ({topic})')

        self.topic = topic

    def publish(self, data: T):
        self.topic.broadcast(data)


topics = {}


class Subscriber:
    def __init__(self, topic_name, topic_type: type, callback: Callable[[T], None]):
        global topics
        if topic_name not in topics:
            topics[topic_name] = Topic(topic_name, topic_type)

        topic = topics[topic_name]
        if topic.topic_type != topic_type:
            raise TypeError(
                f'Tried to create subscriber ({topic_type}) for ({topic})')
        topic.register_observer(callback)


def inspect(topic_name: str, topic_type: type):
    def log(data):
        print(data)
    Subscriber(topic_name, topic_type, log)


def publish_once(topic_name: str, topic_type: type, data: T):
    pub = Publisher(topic_name, topic_type)
    pub.publish(data)


def subscribe(topic_name: str, topic_type: type):
    def wrapper(callback: Callable[[T], None]):
        Subscriber(topic_name, topic_type, callback)
        return callback
    return wrapper
