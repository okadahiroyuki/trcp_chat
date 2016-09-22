#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-
"""
    DoCoMoQaSrv.py

    Service Server for DoCoMo Q&A API

    The project is hosted on GitHub where your could fork the project or report
    issues. Visit https://github.com/roboworks/

    :copyright: (c) 2015 by Hiroyuki Okada, All rights reserved.
    :license: MIT License (MIT), http://www.opensource.org/licenses/MIT
"""
__author__ = 'Hiroyuki Okada'
__version__ = '0.1'
import sys
import time
sys.path.append(".")
import urllib2
import urllib
import json
import rospy
from std_msgs.msg import String

from trcp_chat.msg import DoCoMoQaReq
from trcp_chat.msg import DoCoMoQaRes
from trcp_chat.msg import DoCoMoQaAnswer

from trcp_chat.srv import DoCoMoQa
from trcp_chat.srv import DoCoMoQaResponse


class DoCoMoQaSrv(object):
    """ DoCoMoQaSrv class """
    def __init__(self):
        """ Initializer """

    def run(self):
        """ run ros node """
        # initialize ros node
        rospy.init_node('DocomoQaSrv')
        rospy.loginfo("start DoCoMoQaSrv node")
        service_server = rospy.Service('docomo_qa',DoCoMoQa,self.Qa_handler)
        rospy.loginfo("start DoCoMoQa service server")
        self.APIKEY = rospy.get_param("~APIKEY", "xxxx")
        self.url = rospy.get_param("~qa_url","https://api.apigw.smt.docomo.ne.jp/knowledgeQA/v1/ask?" )
        
        rospy.spin()        

    def Qa_handler(self, query):
        rospy.loginfo("DoCoMoQaSrv Query:%s",query)
        req = query.request
        if req.text == '':
            return DoCoMoQaResponse(success=False)        
        msg = {}
        msg['APIKEY'] = self.APIKEY
        msg['q'] = req.text
        url_value = urllib.urlencode(msg)
        request = urllib2.Request(self.url+url_value)
        try:
            response = urllib2.urlopen(request)
        except Exception as e:
            print e
            return DoCoMoQaResponse(success=False)
        the_page = json.loads(response.read())
        msg = the_page['message']

        res = DoCoMoQaRes()
        res.code = the_page['code']
        res.textForSpeech = msg['textForSpeech']
        res.textForDisplay = msg['textForDisplay']        
        for answer in the_page['answers']:
            ans = DoCoMoQaAnswer()
            ans.rank = answer['rank']
            ans.answerText = answer['answerText']
            ans.linkText = answer['linkText']
            ans.linkUrl = answer['linkUrl']                    
            res.answer.append(ans)

        print the_page
        return DoCoMoQaResponse(success=True, response=res)


if __name__ == '__main__':
    try:
        node = DoCoMoQaSrv()
        node.run()
    except rospy.ROSInterruptException:
        pass
