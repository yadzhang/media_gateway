/*============================================================================
 *Name        : mediaagent.cpp
 *Author      : Zhao Guoshuai
 *Version     :
 *Copyright   : Your copyright notice
 *Description : this class packages the NiceAgent into WebAgent
 and each WebAgent object is a thread
 *============================================================================
 */
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <fstream>

#include "webagent.h"
#include "candidate.h"
#include "libniceconfig.h"
#include "codecconverter.h"
#include "debug.h"
#include "imsagent.h"
#include "rtpconverter.h"
#include "mgdebug.h"
#include "commontype.h"
#include "mediasession.h"
#include <string>
#include <sstream>

using namespace std;
using namespace webrtc;

const int SEND_RTCP_INTERVAL = 188;
#define USE_STUN 0

WebAgent::WebAgent(CodecConverter *cc) {
	this->precnt = 0;
	this->curcnt = 0;
	this->codecConverter = cc;
	this->tid = -1;
	this->audioSSRC = "222222222";
	this->videoSSRC = "333333333";
	this->rtpConverter = NULL;

	this->rtpConverter = new RTPConverter();
	this->isCandidateGatheringDone = false;
	this->isReady = false;
	this->lastCandidateArrived = false;
	this->vPayload = webrtc::PT_VP8_DEFAULT;
	this->aPayload = webrtc::PT_PCMA_DEFAULT;

	this->agentState[0] = NICE_COMPONENT_STATE_DISCONNECTED;
	this->agentState[1] = NICE_COMPONENT_STATE_DISCONNECTED;
	this->peerAgent = NULL;
	this->cryptoToWeb = "pCC0edHOwQOuwX0m4+135nil9xt5Lr9exVF7Gs4m";

	this->mainContext = g_main_context_new();
	this->mainLoop = g_main_loop_new(this->mainContext, FALSE);

#if USE_RELIABLE
	this->agent = nice_agent_new_reliable(g_main_loop_get_context(this->mainLoop), NICE_COMPATIBILITY);
#else
	this->agent = nice_agent_new(g_main_loop_get_context(this->mainLoop), NICE_COMPATIBILITY);
#endif

	nice_agent_set_software(this->agent, "Test, Agent");

//	create the connection between the signals and callback function
	g_signal_connect(G_OBJECT(this->agent), "candidate-gathering-done", G_CALLBACK(candidateGatheringDoneCallback), this);
	g_signal_connect(G_OBJECT(this->agent), "component-state-changed", G_CALLBACK(componentStateChangedCallback), this);
	g_signal_connect(G_OBJECT(this->agent), "new-selected-pair", G_CALLBACK(newSelectedPairCallback), this);
	g_signal_connect(G_OBJECT(this->agent), "new-candidate", G_CALLBACK(newCandidateCallback), this);
	g_signal_connect(G_OBJECT(this->agent), "initial-binding-request-received", G_CALLBACK(initialBindingRequestReceivedCallback), this);

	g_object_set(G_OBJECT(this->agent), "controlling-mode", false, NULL);

//	set the stun server and port
//#if USE_STUN
//	const char *stun_server = "222.200.180.144";
	
	const char *stun_server = "23.21.150.121";
	const char *stun_server_port = "3478";
	
	g_object_set(G_OBJECT (this->agent), "stun-server", STUN_SERVER, NULL);
	g_object_set(G_OBJECT (this->agent), "stun-server-port", atoi(STUN_SERVER_PORT), NULL);
//#endif
}

WebAgent::~WebAgent() {
	cout << "Enter delete WebAgent." << endl;
	g_object_unref(this->agent);
	g_main_loop_unref(this->mainLoop);
	g_main_context_unref(this->mainContext);

	if(this->rtpConverter != NULL)
		delete this->rtpConverter;

	if(this->tid != -1)
		pthread_cancel(this->tid);
	cout << "WebAgent deleted successfully." << endl;
}

void WebAgent::run() {
	printf("In WebAgent::run!\n");

	int ret = pthread_create(&(this->tid), NULL, startICE, this);
	printf("%d\n", ret);
	if (ret != 0) {
		printf("create thread failed!\n");
	} else {
		printf("create thread success!\n");
	}
}

void *WebAgent::startICE(void *args) {
	printf("In startICE!\n");
	WebAgent *mediaAgent = (WebAgent *) args;

	/* step: add one stream, with RTP+RTCP components, to each agent */
	mediaAgent->streamID = nice_agent_add_stream(mediaAgent->agent, 2);
	g_assert(mediaAgent->streamID > 0);

	bool isPortOK = false;
	while (!isPortOK) {
		isPortOK = nice_agent_gather_candidates(mediaAgent->agent, mediaAgent->streamID);
	}

	nice_agent_attach_recv(mediaAgent->agent, mediaAgent->streamID,
			NICE_COMPONENT_TYPE_RTP,
			g_main_loop_get_context(mediaAgent->mainLoop), receiveDataCallback,
			mediaAgent);
	nice_agent_attach_recv(mediaAgent->agent, mediaAgent->streamID,
			NICE_COMPONENT_TYPE_RTCP,
			g_main_loop_get_context(mediaAgent->mainLoop), receiveDataCallback,
			mediaAgent);

//	run mainloop until local candidates are ready
	if (true != mediaAgent->isCandidateGatheringDone) {
		g_main_loop_run(mediaAgent->mainLoop);
		g_assert(true == mediaAgent->isCandidateGatheringDone);
	}

	cout << "Local candidates gathering done!" << endl;
	g_main_loop_run(mediaAgent->mainLoop);
}

void WebAgent::sendrtcp(WebAgent* ptr){
	unsigned char buf[305], len;
	memset(buf, 0, sizeof(buf));
	buf[0] = 0x84;
	buf[1] = 206;
	buf[2] = 0;
	buf[3] = 4;

	buf[7] = 1;
	buf[11] = 0;

	unsigned int sc;
	stringstream ss;
	ss << ptr->videoSSRCFromWeb;
	ss >> sc;

	buf[12] = sc>>24;
	buf[13] = sc>>16;
	buf[14] = sc>>8;
	buf[15] = sc;

	static char a = 2;
	buf[16] = ++a;

	int length = 20;

	cout << "in sendRTCP..." << endl;
	if(ptr->rtpConverter->isReady() && ptr->isReady){
		ptr->rtpConverter->encryptRTCP(buf, length);
		nice_agent_send(ptr->agent, ptr->streamID, NICE_COMPONENT_TYPE_RTP, length, (char*) buf);
	}
}

void WebAgent::receiveDataCallback(NiceAgent *agent, guint streamID, guint componentID, guint len, gchar *buf, gpointer ptr) {
	MG_DEBUG("WebAgent receive data call back!");
	WebAgent *mediaAgent = (WebAgent *) ptr;

	if (componentID == NICE_COMPONENT_TYPE_RTP) {
		MG_DEBUG("Receive from web: %d bytes.", len);

		unsigned char *mediaData = (unsigned char *) buf;
		int length = (int) len;

		if(mediaAgent->rtpConverter->isReady() == false || mediaAgent->isReady == false)return ;
//		if(length < 100)return ;

//		if(mediaAgent->rtpConverter->isReady() == false)return ;
//		if(length < 100)return ;

		if (mediaData[0] == 0x80) {
			RTP_HEADER *srtpHeader = (RTP_HEADER *) &mediaData[0];
			int payload = srtpHeader->payload;

			if (payload == mediaAgent->aPayload) {	//PCMA audio
				MG_DEBUG("Receive audio data From web: %d bytes.", length);

				mediaAgent->curcnt++;
				mediaAgent->rtpConverter->decryptData(mediaData, length);
				mediaAgent->peerAgent->sendData(mediaData, length, 0);
			} else if (payload == mediaAgent->vPayload) {			//VP8 video
				static int cnt = 0;
				if(++cnt == SEND_RTCP_INTERVAL){
					sendrtcp(mediaAgent);
					cnt = 0;
				}

				mediaAgent->curcnt++;
				MG_DEBUG("Receive video data From web: %d bytes.", length);
				mediaAgent->rtpConverter->decryptData(mediaData, length);
				mediaAgent->codecConverter->convertAndSend(mediaData,length);
			}else{
				MG_DEBUG("What mediaData is this?");
			}
		} else {
			MG_DEBUG("Unidentified media data received...");
		}

	} else {
		MG_DEBUG("Receive RTCP: %d bytes.", len);
	}

}

void WebAgent::newCandidateCallback(NiceAgent *agent, guint streamID,
		guint componentID, gchar *foundation, gpointer ptr) {
	string candidate1="candidate=";
	candidate1+=foundation;
	g_print(candidate1.c_str());
	g_print("Find new Candidate Callback!\n");
}

void WebAgent::candidateGatheringDoneCallback(NiceAgent *agent,
		guint streamID, gpointer ptr) {
	g_print("Candidate Gathering Done Callback!\n");
	WebAgent *mediaAgent = (WebAgent *) ptr;
	mediaAgent->isCandidateGatheringDone = true;

	for (int i = 1; i <= 2; ++i) {
		GSList *j = nice_agent_get_local_candidates(mediaAgent->agent,
				mediaAgent->streamID, i);
		for (; j; j = j->next) {
			NiceCandidate *tmp_candidate = (NiceCandidate *) (j->data);
			NiceAddress tmp_addr = tmp_candidate->addr;
			char string_addr[128];
			nice_address_to_string(&tmp_addr, string_addr);

			g_print("string_addr=%s\n",string_addr);
			if (strstr(string_addr, ":") == NULL) {
				NiceAddress tmp_baseaddr = tmp_candidate->base_addr;

				string candidate = "a=candidate:";
				candidate += (tmp_candidate->foundation);
				candidate += ' ';

				char component_id[16];
				sprintf(component_id, "%d ", tmp_candidate->component_id);
				candidate += (component_id);

				if (tmp_candidate->transport == NICE_CANDIDATE_TRANSPORT_UDP) {
					candidate += ("udp ");
				}

				char priority[32];
				sprintf(priority, "%d ", tmp_candidate->priority);
				candidate += (priority);

				nice_address_to_string(&tmp_addr, string_addr);
				candidate += (string_addr);
				candidate += ' ';

				char port[32];
				sprintf(port, "%d ", nice_address_get_port(&tmp_addr));
				candidate += port;

				candidate += ("typ ");

				switch (tmp_candidate->type) {
				case NICE_CANDIDATE_TYPE_HOST:
					candidate += ("host ");
					break;
				case NICE_CANDIDATE_TYPE_SERVER_REFLEXIVE:
					candidate += ("srflx ");
					break;
				case NICE_CANDIDATE_TYPE_PEER_REFLEXIVE:
					candidate += ("prflx ");
					break;
				case NICE_CANDIDATE_TYPE_RELAYED:
				default:
					candidate += ("relayed ");
					break;
				}

				candidate += "generation 0\r\n";
				mediaAgent->localCandidates.push_back(candidate);
			}
		}
	}

	g_main_loop_quit(mediaAgent->mainLoop);
}

/*
 * @Func:  This signal is fired when we received our first binding request from the peer.
 * @params:
 */
void WebAgent::initialBindingRequestReceivedCallback(NiceAgent *agent,
		guint streamID, gpointer ptr) {
	g_print("Initial Binding Request Received!\n");
}

/*
 * @Func:  This signal is fired once a candidate pair is selected for data transfer for a stream's component
 * @params:
 */
void WebAgent::newSelectedPairCallback(NiceAgent *agent, guint streamID,
		guint componentID, gchar *lfoundation, gchar *rfoundation,
		gpointer ptr) {
	g_print("New Selected Pair call back!\n");
}

void WebAgent::componentStateChangedCallback(NiceAgent *agent, guint streamID,
		guint componentID, guint state, gpointer ptr) {
	g_print("Component State change call back!!!");
	WebAgent *mediaAgent = (WebAgent *) ptr;

	mediaAgent->agentState[componentID - 1] = (NiceComponentState) state;
	printf("state of %s component = ", componentID==1 ? "RTP" : "RTCP");
	switch (state) {
	case NICE_COMPONENT_STATE_DISCONNECTED:
		printf("DISCONNECTED\n");
		break;
	case NICE_COMPONENT_STATE_GATHERING:
		printf("GATHERING\n");
		break;
	case NICE_COMPONENT_STATE_CONNECTING:
		printf("CONNECTING\n");
		break;
	case NICE_COMPONENT_STATE_CONNECTED:
		printf("CONNECTED\n");
		break;
	case NICE_COMPONENT_STATE_READY:
		printf("READY\n");
		break;
	case NICE_COMPONENT_STATE_FAILED:
		printf("FAILED\n");
		break;
	case NICE_COMPONENT_STATE_LAST:
		printf("LAST\n");
		break;
	}

	if (state == NICE_COMPONENT_STATE_READY && componentID==1) {
		mediaAgent->isReady = true;
	}
}

/*
 * @Func:  set ice credentials, this is a helper function
 */
void WebAgent::setCredentials(const string &ufrag, const string &pwd) {
	nice_agent_set_remote_credentials(this->agent, this->streamID,
			ufrag.c_str(), pwd.c_str());
}

void WebAgent::getCredentials(string &ufragStr, string &pwdStr) {
	gchar *ufrag = NULL, *pwd = NULL;
	nice_agent_get_local_credentials(this->agent, this->streamID, &ufrag, &pwd);
	ufragStr = string(ufrag);
	pwdStr = string(pwd);
	g_free(ufrag);
	g_free(pwd);
}

void WebAgent::setRemoteCandidates() {
	GSList *cands;

//	set remote RTP candidates
	cands = NULL;
	for(int i=0; i<remoteRTPCandidates.size(); i++){
		Candidate *candidate = new Candidate(remoteRTPCandidates[i].c_str());

		if(candidate->getNiceCandidate()->type == 0)
			cands = g_slist_append(cands, candidate->getNiceCandidate());
		delete candidate;
	}
	nice_agent_set_remote_candidates(this->agent, this->streamID, 1, cands);
	g_slist_free(cands);

//	set remote RTCP candidates
	cands = NULL;
	for(int i=0; i<remoteRTCPCandidates.size(); i++){
		Candidate *candidate = new Candidate(remoteRTCPCandidates[i].c_str());

		if(candidate->getNiceCandidate()->type == 0)
			cands = g_slist_append(cands, candidate->getNiceCandidate());
		delete candidate;
	}
	nice_agent_set_remote_candidates(this->agent, this->streamID, 2, cands);
	g_slist_free(cands);
}

const vector<string> & WebAgent::getLocalCandidates() {
	return this->localCandidates;
}

//mediaData are plain data, not encrypt
void WebAgent::sendData(unsigned char* mediaData, int len, bool type) {
	RTPConverter * con = this->rtpConverter;
	RTP_HEADER *header = (RTP_HEADER *) &mediaData[0];

	if(isReady == 0 || con->isReady()==false)return ;

	if (type == 0) {			//audio data
//		g_assert(header->payload == 8);
//		g_assert(mediaData[0] == 0x80);
		header->ssrc = htonl(atoi(this->audioSSRC.c_str()));
		if(header->payload != this->aPayload || mediaData[0] != 0x80)return ;


		int audioLen = len;
		int encryptResult = con->encryptData(mediaData, audioLen);

		if (this->isReady && encryptResult == 0) {
			MG_DEBUG("Audio data: Sending %d bytes to web.", audioLen);
			nice_agent_send(this->agent, this->streamID, NICE_COMPONENT_TYPE_RTP, audioLen, (char*) mediaData);
		} else {
			//here DEBUG message may be error, since isReady may be false, pay attention
			MG_DEBUG("Encrypt Error message: Audio data won't be sent to Web.");
		}
	} else {
		header->ssrc = htonl(atoi(this->videoSSRC.c_str()));

		header->payload = this->vPayload;
		int encryptResult=con->encryptData(mediaData, len);

		if (this->isReady && encryptResult == 0) {
			MG_DEBUG("Video data: Sending %d bytes to web.", len);
			nice_agent_send(this->agent, this->streamID,
					NICE_COMPONENT_TYPE_RTP, len, (char*) mediaData);
		}else{
			//here DEBUG message may be error, since isReady may be false, pay attention
			MG_DEBUG("Encrypt Error message: Video data won't be sent to Web.");
		}
	}
}

int WebAgent::handleCandidateFromWeb(SessionMessage &sMsg){
	string sdp = sMsg.roap.sdp;
	if(sdp.size() > 0){
		Candidate *candidate = new Candidate(sdp.c_str());

		if(candidate->getNiceCandidate()->component_id == 1){
			this->remoteRTPCandidates.push_back(sdp);
		}else{
			this->remoteRTCPCandidates.push_back(sdp);
		}

		return 0;
	}else{
		lastCandidateArrived = true;
		return 1;
	}
}

bool WebAgent::handleSdpFromWeb(string sdp, bool webCallerFlag, webrtc::SessionType &stype){
	stype = AUDIO;

	if(sdp.find("video") != string::npos){
		stype = VIDEO;
	}

	if (sdp.find("crypto") != string::npos) {
		cryptoFromWeb = getCryptoInfo(sdp);
	}else{
		printf("Error! No crypto in sdp from web!\n");
	}

	string ufrag, pwd;
	getIceInfo(sdp, ufrag, pwd);

	if(ufrag.size() != 0 && pwd.size() != 0){
		setCredentials(ufrag, pwd);
	}

	string::size_type beginPos, endPos, p2;
	const char *st = "m=video";
	beginPos = sdp.find(st);

	if(beginPos != string::npos){

		p2 = sdp.find("a=ssrc:", beginPos);

		if(p2 != string::npos){
			p2 += strlen("a=ssrc:");
			endPos = sdp.find(" ", p2);

			if(endPos != string::npos){
				this->videoSSRCFromWeb = sdp.substr(p2, endPos - p2);
			}
		}
	}

	bool supportFlag = 1;
	beginPos = 0;
	beginPos = sdp.find("m=audio ", beginPos);
	if(beginPos == string::npos){
		supportFlag = 0;
	}else{
		beginPos += strlen("m=audio ");
		//find recv port
		endPos = sdp.find(" ", beginPos);

		beginPos = endPos + 1;

		//find RTP/AVP
		endPos = sdp.find(" ", beginPos);
		beginPos = endPos + 1;

		endPos = sdp.find("\n", beginPos);
		cout << "sy1" << endl;
		while(beginPos < endPos){
			p2 = sdp.find(" ", beginPos);

			if(p2 == string::npos || p2 > endPos){
				p2 = sdp.find("\r\n", beginPos);
				if(sdp.substr(beginPos, p2-beginPos) != "8")
					supportFlag = 0;
				cout << "sy2" << endl;
				break;
			}

			cout << "sy3: " << sdp.substr(beginPos, p2-beginPos) << endl;
			if(sdp.substr(beginPos, p2-beginPos) == "8"){
				break;
			}
			beginPos = p2 + 1;
		}

	}

	cout << "CP1: flag = " << supportFlag << endl;

	beginPos = 0;
	if(stype == VIDEO){
		do{
			beginPos = sdp.find("a=rtpmap:", beginPos);
			if(beginPos == string::npos){
				supportFlag = 0;
				break;
			}
			beginPos += strlen("a=rtpmap:");
			endPos = sdp.find(" ", beginPos);
			string vStr = sdp.substr(beginPos, endPos-beginPos);
			if(sdp.substr(endPos+1, 3) == "VP8"){
				this->vPayload = atoi(vStr.c_str());
				cout << "vPayload = " << this->vPayload << endl;
				break;
			}
		}while(true);
	}

	cout << "CP2: supportFlag = " << supportFlag << endl;
	return supportFlag;
}

void WebAgent::getIceInfo(const string &sdp, string &ufrag, string &pwd){
	string::size_type beginPos, endPos;

	beginPos = sdp.find(ICE_UFRAG);
	if(beginPos != string::npos){
		beginPos += strlen(ICE_UFRAG);
		endPos = sdp.find(PARAM_END, beginPos);
		if(endPos != string::npos){
			ufrag = sdp.substr(beginPos, endPos - beginPos);
		}
	}

	beginPos = sdp.find(ICE_PWD);
	if(beginPos != string::npos){
		beginPos += strlen(ICE_PWD);
		endPos = sdp.find(PARAM_END, beginPos);
		if(endPos != string::npos){
			pwd = sdp.substr(beginPos, endPos - beginPos);
		}
	}
}

string WebAgent::getCryptoInfo(const string &sdp){
	string::size_type beginPos, endPos;
	beginPos = sdp.find(CRYPTO_80);
	if(beginPos != string::npos){
		beginPos += strlen(CRYPTO_80);
		endPos = sdp.find(PARAM_END, beginPos);
		if(endPos != string::npos){
			return sdp.substr(beginPos, endPos - beginPos);
		}
	}
	return "NULL";
}

string WebAgent::createSdpToWeb(SessionType stype, bool webCallerFlag){
	string webSdp = WEB_SDP_BEGIN;

	if(stype == AUDIO){
		webSdp += WEB_AUDIO_BUNDLE;
	}else{
		webSdp += WEB_VIDEO_BUNDLE;
	}

	string aPayloadStr = intToString(this->aPayload);

	webSdp+=    (string)"m=audio 1 RTP/SAVPF " + aPayloadStr +"\r\n"+
						"c=IN IP4 0.0.0.0\r\n"+
						"a=rtcp:1 IN IP4 0.0.0.0\r\n"+
						"a=ice-ufrag:V6oIgATQATtWFDI1\r\n"+
						"a=ice-pwd:0fvI8mFb2+EFTVarDx/RHNvW\r\n"+
						"a=sendrecv\r\n" +
						"a=mid:audio\r\n"+
						"a=rtcp-mux\r\n"+
						"a=crypto:1 AES_CM_128_HMAC_SHA1_80 inline:gnXrE0MSrzWLNu4VzjRm/A4d3ANy0gnpvDNXQlWd\r\n"+
						"a=rtpmap:" + aPayloadStr + " PCMA/8000\r\n";

	string ssrc = audioSSRC;//by songyang
	webSdp += SSRC;
	webSdp += ssrc;
	webSdp += SSRC_END;
	webSdp += "nuGkF01P6jVEX8n8\r\na=ssrc:";
	webSdp += ssrc;
	webSdp += " mslabel:gUzyEDlZWN688rqW3YWb6Al1eqdok5HBdc7M\r\na=ssrc:";
	webSdp += ssrc;
	webSdp += " label:gUzyEDlZWN688rqW3YWb6Al1eqdok5HBdc7M00\r\n";

	if(stype == VIDEO){
		string vPayloadStr = intToString(this->vPayload);

		webSdp  +=   "m=video 1 RTP/SAVPF " + vPayloadStr +"\r\n"+
					  "c=IN IP4 0.0.0.0\r\n"+
					  "a=rtcp:1 IN IP4 0.0.0.0\r\n"+
					  "a=ice-ufrag:V6oIgATQATtWFDI1\r\n"+
					  "a=ice-pwd:0fvI8mFb2+EFTVarDx/RHNvW\r\n" +
					  "a=sendrecv\r\n" +
					  "a=mid:video\r\n"+
					  "a=rtcp-mux\r\n"+
					  "a=crypto:1 AES_CM_128_HMAC_SHA1_80 inline:gnXrE0MSrzWLNu4VzjRm/A4d3ANy0gnpvDNXQlWd\r\n"+
					  "a=rtpmap:" + vPayloadStr + " VP8/90000\r\n";

		ssrc = videoSSRC;//by songyang
		webSdp += SSRC;
		webSdp += ssrc;
		webSdp += SSRC_END;
		webSdp += "nuGkF01P6jVEX8n8\r\na=ssrc:";
		webSdp += ssrc;
		webSdp += " mslabel:gUzyEDlZWN688rqW3YWb6Al1eqdok5HBdc7M\r\na=ssrc:";
		webSdp += ssrc;
		webSdp += " label:gUzyEDlZWN688rqW3YWb6Al1eqdok5HBdc7M10\r\n";

	}

	replaceUfragPwd(webSdp);
	replaceCrypto(webSdp, cryptoToWeb);

	return webSdp;
}

void WebAgent::replaceCrypto(string &sdp, const string & crypto){
	string::size_type begin, end;
	begin = sdp.find(CRYPTO_80);
	while(begin != string::npos){
		begin += strlen(CRYPTO_80);
		end = sdp.find(PARAM_END, begin);
		if(end != string::npos){
			sdp.replace(begin, end - begin, crypto);

		}else{
			cout <<"Replace crypto failed!"<< endl;
			break;
		}
		begin = sdp.find(CRYPTO_80, end);
	}
}

void WebAgent::replaceUfragPwd(string &sdp){
	cout << "Enter replaceUfragPwd!"<< endl;
	string ufrag, pwd;
	getCredentials(ufrag, pwd);

	string::size_type beginPos, endPos;
	beginPos = sdp.find(ICE_UFRAG);
	while(beginPos != string::npos){
		if(beginPos != string::npos){
			beginPos += strlen(ICE_UFRAG);
			endPos = sdp.find(PARAM_END, beginPos);
			if(endPos != string::npos){
				sdp.replace(beginPos, endPos - beginPos, ufrag);
				cout << "Replace the ufrag in sdp to "<< ufrag<< endl;
			}
		}

		beginPos = sdp.find(ICE_PWD, beginPos);
		if(beginPos != string::npos){
			beginPos += strlen(ICE_PWD);
			endPos = sdp.find(PARAM_END, beginPos);
			if(endPos != string::npos){
				sdp.replace(beginPos, endPos - beginPos, pwd);
				cout << "Replace the pwd in sdp to "<< pwd<< endl;
			}
		}
		beginPos = sdp.find(ICE_UFRAG, beginPos);
	}
}

void WebAgent::doTailingJobs(){
	this->rtpConverter->init(this->cryptoToWeb, this->cryptoFromWeb);
}
