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


app = Flask(__name__)

configuration = Configuration(access_token='+yioKpwHVUAks0Nzh2IWxZEWkcr/dR/gaUkSOgKX4rdM6JIp/AA14TXk7omNpycSIYUbULUid7JsrmDFsTJiai3p401IO+mTWM+pTNyVUUTUalgHYn1iAqntFPj8KX/TnHGXjtdA9SxnhIJIeaH9BwdB04t89/1O/w1cDnyilFU=')
handler = WebhookHandler('8f6c355eaf0441621bb8618c750aebc5')

# LL< history
user_history = ["", "", "", "", ""]
assistant_history = ["", "", "", "", ""]
answer = ""

### Get your API keys from openai, you will need to create an account. 
OPENAI_API_VERSION = os.getenv("OPENAI_API_VERSION")
AZURE_OPENAI_ENDPOINT = os.getenv("AZURE_OPENAI_ENDPOINT")
AZURE_OPENAI_API_KEY = os.getenv("AZURE_OPENAI_API_KEY")
AZURE_OPENAI_DEPLOYMENT_NAME = os.getenv("AZURE_OPENAI_DEPLOYMENT_NAME")
AZURE_OPENAI_DEPLOYMENT_NAME_CHAT = os.getenv("AZURE_OPENAI_DEPLOYMENT_NAME_CHAT")

credential = DefaultAzureCredential()

llm = AzureOpenAI(
    api_key=AZURE_OPENAI_API_KEY,
    azure_endpoint=AZURE_OPENAI_ENDPOINT,
    api_version=OPENAI_API_VERSION,
)
### embeding
embeddings = AzureOpenAIEmbeddings(
    azure_deployment=AZURE_OPENAI_DEPLOYMENT_NAME,
    openai_api_version=OPENAI_API_VERSION,#"2023-05-15",
)
docsearch = FAISS.load_local("faiss_index", embeddings,allow_dangerous_deserialization=True)

@app.route('/')
def index():
   print('Request for index page received')
   return render_template('index.html')

@app.route('/favicon.ico')
def favicon():
    return send_from_directory(os.path.join(app.root_path, 'static'),
                               'favicon.ico', mimetype='image/vnd.microsoft.icon')

@app.route('/hello', methods=['POST'])
def hello():
   name = request.form.get('name')

   if name:
       print('Request for hello page received with name=%s' % name)
       return render_template('hello.html', name = name)
   else:
       print('Request for hello page received with no name or blank name -- redirecting')
       return redirect(url_for('index'))

@app.route("/callback", methods=['GET','POST'])
def callback():

    # get X-Line-Signature header value
    signature = request.headers['X-Line-Signature']

    # get request body as text
    body = request.get_data(as_text=True)
    app.logger.info("Request body: " + body)

    # handle webhook body
    try:
        handler.handle(body, signature)
    except InvalidSignatureError:
        app.logger.info("Invalid signature. Please check your channel access token/channel secret.")
        abort(400)

    return 'OK'

@handler.add(MessageEvent, message=TextMessageContent)
def handle_message(event):
    with ApiClient(configuration) as api_client:
        line_bot_api = MessagingApi(api_client)

        result = docsearch.similarity_search(event.message.text)
        response = llm.chat.completions.create(
            model=AZURE_OPENAI_DEPLOYMENT_NAME_CHAT , # model = "deployment_name".
            temperature=0.95,
            messages=[
                {"role": "system", "content": "you are hucenrotia-assistant\n\n"+ str(result[0])},
                {"role": "user", "content": user_history[4]},
                {"role": "assistant", "content": assistant_history[4]},
                {"role": "user", "content": user_history[3]},
                {"role": "assistant", "content": assistant_history[3]},
                {"role": "user", "content": user_history[2]},
                {"role": "assistant", "content": assistant_history[2]},
                {"role": "user", "content": user_history[1]},
                {"role": "assistant", "content": assistant_history[1]},
                {"role": "user", "content": user_history[0]},
                {"role": "assistant", "content": assistant_history[0]},
                {"role": "user", "content": event.message.text}
            ]
        )

        if event.message.text.lower() == "clear":
            answer = "Ok, I will clear our conversation"
            for i in range(len(assistant_history)):
                user_history[i] = ""
                assistant_history[i] = ""
        else:
            for i in range(len(assistant_history)-1):
                user_history[i] = user_history[i + 1]
                assistant_history[i] = assistant_history[i + 1]

            answer = response.choices[0].message.content
            user_history[len(assistant_history) - 1] = event.message.text
            assistant_history[len(assistant_history) - 1] = answer

        line_bot_api.reply_message_with_http_info(
            ReplyMessageRequest(
                reply_token=event.reply_token,
                messages=[TextMessage(text=answer)]
            )
        )

if __name__ == '__main__':
   app.run()
