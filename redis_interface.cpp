/****************************************************************************** 
 * redis_interface.cpp
 *
 * The functions that are used to interface with redis.
 *
 * - sendDataToRedis(hostname,port,value,password): the function that sends data to redis.
 * - getDataFromRedis(hostname,port,key,outValue,password): the function that recieves data from redis.
 *
 ******************************************************************************/ 
#include "redis_interface.h"
#include <iostream>
#include <cstring>
#include <hiredis/hiredis.h>

// function to send data to the Redis server
int sendDataToRedis(const std::string &hostname,
                    int port,
                    const std::string &key,
                    const std::string &value,
                    const std::string &password)
{
    // connects to redis
    redisContext *c = redisConnect(hostname.c_str(), port);
    if (c == nullptr || c->err) {
        if (c) {
            std::cerr << "[ERROR] Connection error: " << c->errstr << std::endl;
            redisFree(c);
        } else {
            std::cerr << "[ERROR] Connection error: can't allocate redis context." << std::endl;
        }
        return -1;
    }

    // sets the password if it is needed
    if (!password.empty()) {
        redisReply *authReply = (redisReply *)redisCommand(c, "AUTH %s", password.c_str());
        if (!authReply || authReply->type == REDIS_REPLY_ERROR) {
            std::cerr << "[ERROR] Redis AUTH failed. Check password or Redis config.";
            if (authReply) {
                std::cerr << " Error: " << authReply->str;
                freeReplyObject(authReply);
            }
            std::cerr << std::endl;
            redisFree(c);
            return -1;
        }
        freeReplyObject(authReply);
    }

    // sends a command to SET data in the server
    redisReply *reply = (redisReply *)redisCommand(c, "SET %s %s", key.c_str(), value.c_str());
    if (!reply) {
        std::cerr << "ERROR: " << c->errstr << std::endl;
        redisFree(c);
        return -1;
    }

    // check if reply successfully sent
    if (reply->type == REDIS_REPLY_STATUS) {
        std::cout << "Redis SET: " << key << " => " << value
                  << " (" << reply->str << ")" << std::endl;
    }

    // free objects and leave program
    freeReplyObject(reply);
    redisFree(c);
    return 0;
}

// function to obtain data from the redis server
int getDataFromRedis(const std::string &hostname,
                     int port,
                     const std::string &key,
                     std::string &outValue,
                     const std::string &password)
{
    // connects to redis
    redisContext *c = redisConnect(hostname.c_str(), port);
    if (c == nullptr || c->err) {
        if (c) {
            std::cerr << "[ERROR] Connection error: " << c->errstr << std::endl;
            redisFree(c);
        } else {
            std::cerr << "[ERROR] Connection error: can't allocate redis context." << std::endl;
        }
        return -1;
    }

    // uses password if that is applicable
    if (!password.empty()) {
        redisReply *authReply = (redisReply *)redisCommand(c, "AUTH %s", password.c_str());
        if (!authReply || authReply->type == REDIS_REPLY_ERROR) {
            std::cerr << "[ERROR] Redis AUTH failed.\n";
            if (authReply) {
                std::cerr << "       Error: " << authReply->str << std::endl;
                freeReplyObject(authReply);
            }
            redisFree(c);
            return -1;
        }
        freeReplyObject(authReply);
    }

    // sends the GET command to the redis server to obtain data
    redisReply *reply = (redisReply *)redisCommand(c, "GET %s", key.c_str());
    if (!reply) {
        std::cerr << "[ERROR] Command error: " << c->errstr << std::endl;
        redisFree(c);
        return -1;
    }

    // checks if the reply string was successfuly sent
    if (reply->type == REDIS_REPLY_STRING) {
        outValue = reply->str;
    } else if (reply->type == REDIS_REPLY_NIL) {
        outValue.clear();
    } else {
        outValue.clear();
    }

    // free objects and clost function
    freeReplyObject(reply);
    redisFree(c);
    return 0;
}
