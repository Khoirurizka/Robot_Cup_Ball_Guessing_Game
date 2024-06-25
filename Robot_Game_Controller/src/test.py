from flask import Flask

# Create a Flask application

# Define a class for your routes
class MyRoutes:
    def __init__(self):
        self.app = Flask(__name__)

        # Register routes
        self.register_routes()

    def register_routes(self):
        # Define your routes as methods of this class
        @self.app.route('/')
        def index():
            return 'Hello, World!'

        @self.app.route('/about')
        def about():
            return 'About page'

# Initialize and run the application
if __name__ == '__main__':
    routes = MyRoutes()
    routes.app.run(debug=True)
