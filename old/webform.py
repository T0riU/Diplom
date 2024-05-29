import webview
from jinja2 import Environment, FileSystemLoader, select_autoescape
import threading
import time

file_name = 'hud.html'

def open_gui():
    env = Environment(
        loader=FileSystemLoader('.'),
        autoescape=select_autoescape(['html', 'xml'])
    )
    template = env.get_template(file_name)
    html_content = template.render()
    window = webview.create_window("HTML", html=html_content, width=750, height=750, resizable=False, fullscreen=False)

    def call_update_js():
        number = 45
        try:
            while True:
                time.sleep(1)
                if number > 350:
                    number = 0
                number += 5
                window.evaluate_js('updateJS({})'.format(number))
        except webview.WindowDestroyed:
            pass  # Exit the loop when the window is closed

    threading.Thread(target=call_update_js).start()
    webview.start()

if __name__ == '__main__':
    open_gui()