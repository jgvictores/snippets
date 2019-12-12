#!/usr/bin/env python

from HTMLParser import HTMLParser

messages_file_name = 'messages.html'

class MyHTMLParser(HTMLParser):
    def __init__(self):
        HTMLParser.__init__(self)
        self.nextIsGood = False

    def handle_starttag(self, tag, attrs):
        if tag == "div":
            for attr in attrs:
                if (attr[0] == 'title'):
                    self.fechaHora = attr[1]
                if (attr[0] == 'class') & (attr[1] == 'text'):
                     self.nextIsGood = True

    def handle_data(self, data):
        if self.nextIsGood == True:
            lines = data.splitlines()
            for line in lines:
                line = line.strip()
                if line:
                    fecha, hora = self.fechaHora.split(' ')
                    fecha = fecha.replace(".", "/")
                    print(fecha+", "+hora+", "+line)
            self.nextIsGood = False

parser = MyHTMLParser()
messages_file = open(messages_file_name, 'r')
parser.feed(messages_file.read())
messages_file.close()
