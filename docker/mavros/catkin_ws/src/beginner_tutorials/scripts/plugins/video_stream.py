#!/usr/bin/python3
# -*- coding: utf-8 -*-
import abc

import cv2


class BaseVideoStream(abc.ABC):
    pass


class Mp4VideoStream(BaseVideoStream):
    def __init__(self, source: str):
        super(Mp4VideoStream, self).__init__()
        if type(source) is not str:
            raise ValueError(f"[{self.__class__.__name__}] type of argument `source` must be 'str'.")
        self._source = source

    def __iter__(self):
        self._capture = cv2.VideoCapture(filename=self.source)
        if not self.capture.isOpened():
            self.capture.open()
        return self

    def __next__(self):
        ret, frame = self.capture.read()
        if ret:
            return frame
        self.capture.release()
        raise StopIteration()

    def release(self):
        self.capture.release()

    @property
    def capture(self) -> cv2.VideoCapture:
        return self._capture

    @property
    def source(self) -> str:
        return self._source
