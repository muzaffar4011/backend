# Chatbot Troubleshooting Guide (Hindi/Urdu)

## समस्या: "Cannot connect to backend API" या "Failed to fetch details" Error

अगर chatbot काम नहीं कर रहा है और आपको "Cannot connect to backend API" या "Failed to fetch details" error मिल रहा है, तो निम्नलिखित steps follow करें:

---

## ✅ Step 1: Backend Server Check करें

**Backend server चल रहा है या नहीं, यह check करें:**

1. **Terminal खोलें** और `backend` folder में जाएं:
   ```bash
   cd backend
   ```

2. **Virtual environment activate करें:**
   ```bash
   # Windows
   venv\Scripts\activate
   # या
   myenv\Scripts\activate
   ```

3. **Backend server start करें:**
   ```bash
   uvicorn main:app --reload --host localhost --port 8000
   ```

4. **Expected output:**
   ```
   INFO:     Uvicorn running on http://localhost:8000 (Press CTRL+C to quit)
   INFO:     Application startup complete.
   ```

**⚠️ अगर error आ रहा है:**
- Port 8000 already in use: किसी और port use करें या पुराना process kill करें
- Module not found: `pip install -r requirements.txt` run करें
- Database error: `.env` file में `DATABASE_URL` check करें

---

## ✅ Step 2: Browser में API Check करें

**Browser खोलें और यह URL open करें:**

```
http://localhost:8000/api/health
```

**Expected response:**
```json
{
  "status": "healthy",
  "timestamp": "2025-12-07T..."
}
```

**अगर error आ रहा है:**
- "Connection refused" या "ERR_CONNECTION_REFUSED": Backend server नहीं चल रहा है
- "404 Not Found": URL गलत है या route नहीं है
- "CORS error": `.env` file में `CORS_ORIGINS` check करें

---

## ✅ Step 3: CORS Configuration Check करें

**`.env` file check करें** (backend folder में):

```env
CORS_ORIGINS=http://localhost:3000
```

**Important:**
- Frontend `http://localhost:3000` पर चल रहा होना चाहिए
- अगर frontend किसी और port पर है, तो उसे भी add करें:
  ```env
  CORS_ORIGINS=http://localhost:3000,http://localhost:5173
  ```
- Multiple origins को comma (`,`) से separate करें
- Spaces नहीं होने चाहिए

**Backend restart करें** `.env` file change करने के बाद!

---

## ✅ Step 4: Frontend API URL Check करें

**File check करें:** `book_frontend/src/plugins/rag-chatbot/api/chatClient.ts`

**Line 6-8 check करें:**
```typescript
const API_BASE_URL = process.env.NODE_ENV === 'production'
  ? 'https://rag-chatbot-api-lr57.onrender.com'
  : 'http://localhost:8000';
```

**Development mode में `http://localhost:8000` होना चाहिए!**

---

## ✅ Step 5: Browser Console Check करें

1. **Browser में F12 दबाएं** (Developer Tools खोलने के लिए)
2. **Console tab** पर जाएं
3. **Chatbot open करें** और error देखें

**Common errors:**
- `TypeError: Failed to fetch`: Backend server नहीं चल रहा या CORS issue है
- `CORS policy error`: Backend में CORS configuration गलत है
- `404 Not Found`: API endpoint गलत है
- `400 Bad Request`: Request format गलत है

---

## ✅ Step 6: Network Tab Check करें

1. **Browser में F12 दबाएं**
2. **Network tab** पर जाएं
3. **Chatbot में message send करें**
4. **Request check करें:**

**Expected:**
- Request URL: `http://localhost:8000/api/session` या `/api/chat`
- Status: `200 OK` या `201 Created`
- Headers में `Access-Control-Allow-Origin` होना चाहिए

**Problems:**
- Status `400`: OPTIONS request fail हो रहा है (CORS issue)
- Status `0` या `(failed)`: Server connection नहीं हो रहा
- Status `404`: Wrong endpoint
- Status `500`: Server-side error (backend logs check करें)

---

## ✅ Step 7: Environment Variables Check करें

**Backend `.env` file में ये variables होने चाहिए:**

```env
# Database
DATABASE_URL=postgresql://user:password@localhost:5432/rag_chatbot

# OpenAI
OPENAI_API_KEY=sk-...

# Qdrant
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-api-key

# CORS
CORS_ORIGINS=http://localhost:3000

# Optional
SENTRY_DSN=...
```

**Check करें:**
- सभी required variables present हैं
- Values empty नहीं हैं
- Special characters properly URL-encoded हैं (password में)

---

## ✅ Step 8: Ports Check करें

**Check करें कि ports free हैं:**

```bash
# Windows PowerShell
netstat -ano | findstr :8000
netstat -ano | findstr :3000
```

**अगर port in use है:**
1. Process ID (PID) note करें
2. Task Manager में जाएं और process kill करें
3. या different port use करें

---

## ✅ Step 9: Complete Restart करें

**अगर कुछ भी काम नहीं कर रहा:**

1. **Backend stop करें** (Ctrl+C)
2. **Frontend stop करें** (Ctrl+C)
3. **Wait करें 5 seconds**
4. **Backend start करें:**
   ```bash
   cd backend
   venv\Scripts\activate  # या myenv\Scripts\activate
   uvicorn main:app --reload --host localhost --port 8000
   ```
5. **Frontend start करें** (दूसरे terminal में):
   ```bash
   cd book_frontend
   npm start
   ```
6. **Browser refresh करें** (Ctrl+F5)

---

## 🔍 Common Issues और Solutions

### Issue 1: "Cannot connect to backend API at http://localhost:8000"
**Solution:**
- Backend server check करें - क्या वह चल रहा है?
- Browser में `http://localhost:8000/api/health` open करें
- Firewall check करें

### Issue 2: "CORS policy" error
**Solution:**
- `.env` file में `CORS_ORIGINS` check करें
- Frontend URL correct होना चाहिए
- Backend restart करें

### Issue 3: "Failed to create session" error
**Solution:**
- Database connection check करें
- `.env` में `DATABASE_URL` verify करें
- Database running होनी चाहिए

### Issue 4: Backend starts but crashes on request
**Solution:**
- Backend terminal में error logs check करें
- Missing dependencies: `pip install -r requirements.txt`
- Database migration: `alembic upgrade head`

### Issue 5: Frontend builds but chatbot doesn't appear
**Solution:**
- Browser console check करें
- Plugin properly installed है? `docusaurus.config.ts` check करें
- Browser cache clear करें

---

## 📝 Quick Checklist

Before asking for help, make sure:

- [ ] Backend server running है (`http://localhost:8000/api/health` works)
- [ ] Frontend server running है (`http://localhost:3000` opens)
- [ ] `.env` file में सभी variables correct हैं
- [ ] `CORS_ORIGINS=http://localhost:3000` set है
- [ ] Browser console में कोई obvious error नहीं है
- [ ] Network tab में requests `200 OK` हैं
- [ ] Database connection working है
- [ ] All dependencies installed हैं (`pip install -r requirements.txt`)

---

## 🆘 Still Not Working?

अगर ये सब करने के बाद भी काम नहीं कर रहा:

1. **Complete error message copy करें** (browser console से)
2. **Backend terminal output copy करें**
3. **`.env` file check करें** (passwords/keys नहीं share करें!)
4. **Operating System और Node/Python versions note करें**

---

## 📞 Quick Test Commands

**Terminal में ये commands run करके test करें:**

```bash
# 1. Backend health check
curl http://localhost:8000/api/health

# 2. Test session creation
curl -X POST http://localhost:8000/api/session \
  -H "Content-Type: application/json"

# 3. Check if port is listening
netstat -ano | findstr :8000
```

**Expected:**
- Command 1: `{"status":"healthy",...}`
- Command 2: `{"session_id":"...","created_at":"..."}`
- Command 3: Port 8000 listed होना चाहिए

---

**Good luck! 🚀**

