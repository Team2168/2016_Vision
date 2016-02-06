import pymjpeg
import time
import thread
import time

from BaseHTTPServer import HTTPServer, BaseHTTPRequestHandler


class MyHandler(BaseHTTPRequestHandler):

    prev = []

    def do_GET(self):
        self.send_response(200)
        # Response headers (multipart)
        for k, v in pymjpeg.request_headers().items():
            self.send_header(k, v) 
        # Multipart content

        while True:
	    # Part boundary string
	    self.end_headers()
	    self.wfile.write(pymjpeg.boundary)
	    self.end_headers()
	    # Part headers
		
	    send = True

	    for k, v in pymjpeg.image_headers('../stream.jpg').items():
	        self.send_header(k, v)
		if v == 0:
			send = False
	    self.end_headers()
	    # Part binary
	    if send:
		    for chunk in pymjpeg.image('../stream.jpg'):
			self.wfile.write(chunk)
    def log_message(self, format, *args):
        return


def run():
    httpd = HTTPServer(('', 8001), MyHandler)
    httpd.serve_forever()

if __name__ == '__main__':
	thread.start_new_thread(run, ())
	while True:
		pass