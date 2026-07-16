import java.util.Properties

plugins {
    id("com.android.application")
}

android {
    namespace = "com.example.robotcaregiver"
    compileSdk = 34

    defaultConfig {
        applicationId = "com.example.robotcaregiver"
        minSdk = 31
        targetSdk = 34
        versionCode = 1
        versionName = "0.1"

        // Load local.properties safely using Kotlin syntax
        val props = Properties()
        val localPropertiesFile = rootProject.file("local.properties")
        if (localPropertiesFile.exists()) {
            localPropertiesFile.inputStream().use { stream ->
                props.load(stream)
            }
        }
        val key = props.getProperty("GEMINI_API_KEY", "")
        val shared = props.getProperty("APP_SHARED", "")
        val backendIp = props.getProperty("BACKEND_IP", "")
        val kitMac = props.getProperty("KIT_BT_MAC", "")


        // Pass the string securely to BuildConfig
        buildConfigField("String", "GEMINI_API_KEY", "\"$key\"")
        buildConfigField("String", "APP_SHARED", "\"$shared\"")
        buildConfigField("String", "BACKEND_IP", "\"$backendIp\"")
        buildConfigField("String", "KIT_BT_MAC", "\"$kitMac\"")
    }

    buildFeatures {
        buildConfig = true
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
}

dependencies {
    implementation("androidx.appcompat:appcompat:1.7.0")
    implementation("com.squareup.okhttp3:okhttp:4.12.0")
    implementation("com.google.android.material:material:1.12.0")
}
