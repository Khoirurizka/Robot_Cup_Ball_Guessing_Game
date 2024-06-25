#!/usr/bin/env python3

import threading
import os

from flask import (Flask, redirect, render_template, request, abort,
                   send_from_directory, url_for)

from linebot.v3 import (
    WebhookHandler
)
from linebot.v3.exceptions import (
    InvalidSignatureError
)
from linebot.v3.messaging import (
    Configuration,
    ApiClient,
    MessagingApi,
    ReplyMessageRequest,
    TextMessage
)
from linebot.v3.webhooks import (
    MessageEvent,
    TextMessageContent
)
from openai import AzureOpenAI
from azure.identity import DefaultAzureCredential, ChainedTokenCredential, ManagedIdentityCredential, AzureCliCredential

from langchain_community.embeddings.openai import OpenAIEmbeddings
from langchain.text_splitter import CharacterTextSplitter
from langchain_community.vectorstores import ElasticVectorSearch, Pinecone, Weaviate, FAISS
import os
from langchain.chains.question_answering import load_qa_chain
#from langchain.llms import OpenAI
from langchain_openai import AzureOpenAIEmbeddings,OpenAIEmbeddings
from langchain.prompts import PromptTemplate
from langchain.chains import LLMChain
from langchain_anthropic import ChatAnthropic
from langchain_core.output_parsers import StrOutputParser
from langchain_core.prompts import ChatPromptTemplate



class task_llm(threading.Thread):

    def __init__(self,name):
        threading.Thread.__init__(self)
        ### web config
        self.app = Flask(__name__)
        self.configuration = Configuration(access_token='+yioKpwHVUAks0Nzh2IWxZEWkcr/dR/gaUkSOgKX4rdM6JIp/AA14TXk7omNpycSIYUbULUid7JsrmDFsTJiai3p401IO+mTWM+pTNyVUUTUalgHYn1iAqntFPj8KX/TnHGXjtdA9SxnhIJIeaH9BwdB04t89/1O/w1cDnyilFU=')
        self.handler = WebhookHandler('8f6c355eaf0441621bb8618c750aebc5')
        self.setup_routes()
        
        ### LLM history
        self.user_history = ["", "", "", "", ""]
        self.assistant_history = ["", "", "", "", ""]
        self.answer = ""

        ### Get your API keys from openai, you will need to create an account. 
        self.OPENAI_API_VERSION = os.getenv("OPENAI_API_VERSION")
        self.AZURE_OPENAI_ENDPOINT = os.getenv("AZURE_OPENAI_ENDPOINT")
        self.AZURE_OPENAI_API_KEY = os.getenv("AZURE_OPENAI_API_KEY")
        self.AZURE_OPENAI_DEPLOYMENT_NAME = os.getenv("AZURE_OPENAI_DEPLOYMENT_NAME")
        self.AZURE_OPENAI_DEPLOYMENT_NAME_CHAT = os.getenv("AZURE_OPENAI_DEPLOYMENT_NAME_CHAT")

        self.credential = DefaultAzureCredential()

        self.llm = AzureOpenAI(
            api_key=self.AZURE_OPENAI_API_KEY,
            azure_endpoint=self.AZURE_OPENAI_ENDPOINT,
            api_version=self.OPENAI_API_VERSION,
        )
        ### embeding
        self.embeddings = AzureOpenAIEmbeddings(
            azure_deployment=self.AZURE_OPENAI_DEPLOYMENT_NAME,
            openai_api_version=self.OPENAI_API_VERSION,#"2023-05-15",
        )
        # get the current working directory
        self.current_working_directory = os.getcwd()

        ### !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ###
        ### !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ###
        print("root:= "+self.current_working_directory)
        #self.docsearch = FAISS.load_local("faiss_index", self.embeddings,allow_dangerous_deserialization=True)
        self.docsearch = FAISS.load_local("src/Robot_Cup_Ball_Guessing_Game/Robot_Game_Controller/src/faiss_index", self.embeddings,allow_dangerous_deserialization=True)
        
    def setup_routes(self):
        @self.app.route('/')
        def index():
           print('Request for index page received')
           return render_template('index.html')

        @self.app.route('/favicon.ico')
        def favicon():
            return send_from_directory(os.path.join(app.root_path, 'static'),
                               'favicon.ico', mimetype='image/vnd.microsoft.icon')

        @self.app.route('/hello', methods=['POST'])
        def hello():
           name = request.form.get('name')

           if name:
               print('Request for hello page received with name=%s' % name)
               return render_template('hello.html', name = name)
           else:
               print('Request for hello page received with no name or blank name -- redirecting')
               return redirect(url_for('index'))

        @self.app.route("/callback", methods=['GET','POST'])
        def callback():

            # get X-Line-Signature header value
            self.signature = request.headers['X-Line-Signature']
    
            # get request body as text
            self.body = request.get_data(as_text=True)
            self.app.logger.info("Request body: " + self.body)

            # handle webhook body
            try:
                self.handler.handle(self.body, self.signature)
            except InvalidSignatureError:
                self.app.logger.info("Invalid signature. Please check your channel access token/channel secret.")
                abort(400)

            return 'OK'

        @self.handler.add(MessageEvent, message=TextMessageContent)
        def handle_message(event):

            with ApiClient(self.configuration) as api_client:
                self.line_bot_api = MessagingApi(api_client)
                
                self.result = self.docsearch.similarity_search(event.message.text)
            
                self.response = self.llm.chat.completions.create(
                    model=self.AZURE_OPENAI_DEPLOYMENT_NAME_CHAT , # model = "deployment_name".
                    temperature=0.95,
                    messages=[
                        {"role": "system", "content": "you are hucenrotia-assistant\n\n"+ str(self.result[0])},
                        {"role": "user", "content": self.user_history[4]},
                        {"role": "assistant", "content": self.assistant_history[4]},
                        {"role": "user", "content": self.user_history[3]},
                        {"role": "assistant", "content": self.assistant_history[3]},
                        {"role": "user", "content": self.user_history[2]},
                        {"role": "assistant", "content": self.assistant_history[2]},
                        {"role": "user", "content": self.user_history[1]},
                        {"role": "assistant", "content": self.assistant_history[1]},
                        {"role": "user", "content": self.user_history[0]},
                        {"role": "assistant", "content": self.assistant_history[0]},
                        {"role": "user", "content": event.message.text}
                    ]
                )
                #print(self.response.choices[0].message.content)
                #self.answer = self.response.choices[0].message.content
                
                if event.message.text.lower() == "clear":
                    self.answer = "Ok, I will clear our conversation"
                    for self.i in range(len(self.assistant_history)):
                        self.user_history[self.i] = ""
                        self.assistant_history[self.i] = ""
                else:
                    for self.i in range(len(self.assistant_history)-1):
                        self.user_history[self.i] = self.user_history[self.i + 1]
                        self.assistant_history[self.i] = self.assistant_history[self.i + 1]

                    self.answer = self.response.choices[0].message.content
                    self.user_history[len(self.assistant_history) - 1] = event.message.text
                    self.assistant_history[len(self.assistant_history) - 1] = self.answer
                
                
                self.line_bot_api.reply_message_with_http_info(
                    ReplyMessageRequest(
                        reply_token=event.reply_token,
                        messages=[TextMessage(text=self.answer)]
                    )
                )
            return 'OK'

         
    def run(self):
        try:
            print("Task 2 assigned to thread: {}".format(threading.current_thread().name))
            self.app.run(host='127.0.0.3', port=5555,debug=False,threaded=True) 
        except KeyboardInterrupt:
            print("Flask server shutting down...")
            sys.exit(0)


