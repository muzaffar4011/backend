# 502 Bad Gateway - /api/chat Endpoint Fix

## Problem
Browser console shows:
1. **CORS Error**: `Access-Control-Allow-Origin` header missing
2. **502 Bad Gateway**: `POST /api/chat` request failing

Backend logs show:
- ✅ `/api/session` requests successful
- ✅ `OPTIONS /api/chat` requests successful  
- ❌ `/api/chat` POST requests have no response logs (crashes)

## Root Cause
The `/api/chat` endpoint is crashing during RAG processing. Possible causes:
1. **RAG service initialization failing** (Qdrant/OpenAI connection)
2. **Request timeout** (Render free tier has limits)
3. **Memory limit exceeded** (free tier limits)
4. **Unhandled exception** during embedding/LLM generation

---

## ✅ Solution Steps

### Step 1: Check Render Logs for Detailed Errors

1. Go to Render Dashboard: https://dashboard.render.com/
2. Click service: `rag-chatbot-api`
3. Click **Logs** tab
4. Look for errors after `POST /api/chat` request

**Look for:**
- `RAG service error:` - RAG initialization/connection issue
- `Error processing chat query:` - General processing error
- `ConnectionError` - Qdrant/OpenAI connection failed
- `Timeout` - Request taking too long
- `MemoryError` - Out of memory on free tier

### Step 2: Verify Environment Variables

Render Dashboard → Environment tab → Check:

**Required:**
- ✅ `OPENAI_API_KEY` - Must be valid
- ✅ `QDRANT_URL` - Must be correct Qdrant cluster URL
- ✅ `QDRANT_API_KEY` - Must be valid
- ✅ `DATABASE_URL` - Must be working

### Step 3: Test Qdrant Connection

If RAG service fails, verify Qdrant:

```python
# Test locally or add to backend
from qdrant_client import QdrantClient

client = QdrantClient(
    url="YOUR_QDRANT_URL",
    api_key="YOUR_QDRANT_API_KEY"
)

collections = client.get_collections()
print(collections)
```

### Step 4: Check Render Service Limits

Render free tier has limits:
- **Memory**: 512MB
- **CPU**: Shared
- **Request timeout**: ~30 seconds (can be increased)

If RAG processing takes too long or uses too much memory, upgrade plan.

---

## 🔍 Debugging

### Check Latest Render Logs

After making a chat request, check logs for:
```
INFO: Processing query for session: <uuid>
INFO: Starting RAG answer generation...
ERROR: RAG service error: <error details>
```

### Common Errors and Fixes

#### Error 1: Qdrant Connection Failed
```
ConnectionError: Failed to connect to Qdrant
```
**Fix**: 
- Verify `QDRANT_URL` and `QDRANT_API_KEY`
- Check Qdrant cluster is running
- Verify network connectivity from Render

#### Error 2: OpenAI API Error
```
OpenAI API error: Invalid API key
```
**Fix**:
- Verify `OPENAI_API_KEY` is correct
- Check OpenAI account has credits
- Verify API key has proper permissions

#### Error 3: Request Timeout
```
Timeout: Request took too long
```
**Fix**:
- RAG processing may be slow on free tier
- Consider reducing embedding model complexity
- Upgrade to paid Render plan for better resources

#### Error 4: Memory Error
```
MemoryError: Out of memory
```
**Fix**:
- Free tier has 512MB limit
- Reduce batch size for embeddings
- Upgrade Render plan

---

## 📋 Quick Checklist

- [ ] Render logs checked for specific error
- [ ] Environment variables verified
- [ ] Qdrant connection tested
- [ ] OpenAI API key valid
- [ ] Service restarted after changes
- [ ] Latest code deployed (with improved logging)

---

## 🚀 Code Changes Made

1. **Added detailed logging** in `/api/chat` endpoint:
   - Logs before RAG call
   - Logs after RAG call
   - Better error messages

2. **Improved error handling**:
   - Catches RAG-specific errors
   - Better error messages in responses
   - Preserves CORS headers on errors

---

## 📝 Next Steps

1. **Push code changes**:
   ```powershell
   git add backend/routers/chat.py
   git commit -m "Add detailed logging for /api/chat endpoint"
   git push origin main
   ```

2. **Wait for Render deployment** (~3-5 minutes)

3. **Test chatbot** and check logs

4. **Share error logs** if issue persists

---

## 🎯 Expected Result

After fix:
- ✅ Detailed error logs in Render
- ✅ Clear error messages identifying issue
- ✅ CORS headers preserved on errors
- ✅ Better debugging information

---

**Debug Info Added!** 🔍

The improved logging will help identify the exact cause of the 502 error.
